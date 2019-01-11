# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import bus
import logging
import kinematics.extruder

CHIP_ADDR = 0x75
PAT9125_PRODUCT_ID = (0x91 << 8) | 0x31
FLAG_MOTION = 0x80

PAT9125_REGS = {
    'PID1': 0x00, 'PID2': 0x01, 'MOTION': 0x02,
    'DELTA_XL': 0x03, 'DELTA_YL': 0x04, 'MODE': 0x05,
    'CONFIG': 0x06, 'WP': 0x09, 'SLEEP1': 0x0a,
    'SLEEP2': 0x0b, 'RES_X': 0x0d, 'RES_Y': 0x0e,
    'DELTA_XYH': 0x12, 'SHUTTER': 0x14, 'FRAME': 0x17,
    'ORIENTATION': 0x19, 'BANK_SELECTION': 0x7f
}

PAT9125_INIT1 = [
    PAT9125_REGS['WP'], 0x5a,
    PAT9125_REGS['RES_X'], 0,
    PAT9125_REGS['RES_Y'], 240,
    PAT9125_REGS['ORIENTATION'], 0x04,
    0x5e, 0x08, 0x20, 0x64,
    0x2b, 0x6d, 0x32, 0x2f
]

PAT9125_INIT2 = [
    [0x06, 0x28, 0x33, 0xd0, 0x36, 0xc2, 0x3e, 0x01,
     0x3f, 0x15, 0x41, 0x32, 0x42, 0x3b, 0x43, 0xf2],
    [0x44, 0x3b, 0x45, 0xf2, 0x46, 0x22, 0x47, 0x3b,
     0x48, 0xf2, 0x49, 0x3b, 0x4a, 0xf0, 0x58, 0x98],
    [0x59, 0x0c, 0x5a, 0x08, 0x5b, 0x0c, 0x5c, 0x08,
     0x61, 0x10, 0x67, 0x9b, 0x6e, 0x22, 0x71, 0x07],
    [0x72, 0x08]
]

def check_twos_complement(val, bitsize):
    if val & (1 << (bitsize - 1)):
        val -= (1 << bitsize)
    return val

# I2C communications layer for pat9125
class PAT9125_I2C:
    RETRY_TIME = .25
    TIMEOUT = .75
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=CHIP_ADDR, default_speed=400000)
        self.mcu = self.i2c.get_mcu()

        self.oid = self.mcu.create_oid()
        self.get_i2c_oid = self.i2c.get_oid
        self.get_command_queue = self.i2c.get_command_queue
        self.get_mcu = self.i2c.get_mcu
        self.pat9125_write_verify_cmd = None

        self.last_query_time = 0.
        self.write_verify_response = None
        self.write_verify_request = None
        self.write_verify_timer = self.reactor.register_timer(
            self._write_verify_request_event)
    def build_config(self):
        self.pat9125_write_verify_cmd = self.mcu.lookup_command(
            "command_pat9125_write_verify oid=%c sequence=%*s retries=%u",
            cq=self.get_command_queue())
        self.mcu.register_msg(
            self._response_handler, "pat9125_verify_response", self.oid)
    def _write_verify_request_event(self, eventtime):
        if self.write_verify_response is not None:
            return self.reactor.NEVER
        self.pat9125_write_verify_cmd.send(
            self.write_verify_request)
        return eventtime + self.RETRY_TIME
    def _response_handler(self, params):
        if params['#sent_time'] >= self.last_query_time:
            self.write_verify_response = params['success']
        elif params['#sent_time'] < .000001:
            self.write_verify_response = params['success']
            logging.info(
                "PAT9125_I2C: sent time set to zero, allowing")
    def get_oid(self):
        return self.oid
    def read_register(self, reg, read_len):
        # return data from from a register. reg may be a named
        # register, an 8-bit address, or a list containing the address
        if type(reg) is str:
            regs = [PAT9125_REGS[reg]]
        elif type(reg) is list:
            regs = list(reg)
        else:
            regs = [reg]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])
    def write_register(self, reg, data, minclock=0, reqclock=0):
        # Write data to a register. Reg may be a named
        # register or an 8-bit address.  Data may be a list
        # of 8-bit values, or a single 8-bit value
        if type(data) is not list:
            out_data = [data & 0xFF]
        else:
            out_data = list(data)

        if type(reg) is str:
            out_data.insert(0, PAT9125_REGS[reg])
        elif type(reg) is list:
            out_data = reg + out_data
        else:
            out_data.insert(0, reg)
        self.i2c.i2c_write(out_data, minclock, reqclock)
    def write_verify_sequence(self, sequence, retries=5):
        # do a register read/write.  We need to handle our own response
        # because 'send_with_response' uses a different command queue
        # that alters the order of commands sent
        self.write_verify_response = None
        self.write_verify_request = [self.oid, sequence, retries]
        self.reactor.update_timer(self.write_verify_timer, self.reactor.NOW)
        eventtime = self.last_query_time = self.reactor.monotonic()
        while self.write_verify_response is None:
            eventtime = self.reactor.pause(eventtime + .005)
            if eventtime > self.last_query_time + self.TIMEOUT:
                break
        self.reactor.update_timer(self.write_verify_timer, self.reactor.NEVER)
        if self.write_verify_response is not None:
            return self.write_verify_response
        else:
            logging.info("PAT9125: Timeout waiting for response")
            return False

class PAT9125:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.pat9125_i2c = PAT9125_I2C(config)
        self.mcu = self.pat9125_i2c.get_mcu()
        self.mcu.register_config_callback(self.build_config)
        self.cmd_queue = self.pat9125_i2c.get_command_queue()
        self.oid = self.pat9125_i2c.get_oid()
        self.i2c_oid = self.pat9125_i2c.get_i2c_oid()
        self.printer.register_event_handler(
            "klippy:ready", self.handle_ready)

        self.stepper_oid = None
        self.pat9125_query_cmd = None
        self.pat9125_stop_query_cmd = None

        self.e_step_dist = 1.
        self.refresh_time = config.getfloat("refresh_time", .03, above=0.)
        self.invert_axis = config.getboolean("invert_axis", False)
        orient_choices = {'X': 0, 'Y': 1}
        self.orientation = config.getchoice(
            "orientation", orient_choices, 'Y')
        self.xres = 0 if self.orientation else 240
        self.yres = 240 if self.orientation else 0
        PAT9125_INIT1[3] = self.xres
        PAT9125_INIT1[5] = self.yres

        self.watchdog = WatchDog(config, self)
        self.set_callbacks = self.watchdog.set_callbacks
        self.set_insert_enable = self.watchdog.set_insert_enable
        self.set_runout_enable = self.watchdog.set_runout_enable

        self.initialized = False
        self.pat9125_state = {
            'STEPPER_POS': 0.,
            'X_POS': 0,
            'Y_POS': 0,
            'FRAME': 0,
            'SHUTTER': 0,
            'MCU_TIME': 0.
        }
        # XXX ------TEST COMMANDS TO BE REMOVED---------------
        self.gcode.register_command(
            'PAT_READ_ID', self.cmd_PAT_READ_ID)
        self.gcode.register_command(
            'PAT_TEST_INIT', self.cmd_PAT_TEST_INIT)
        self.gcode.register_command(
            'PAT_READ_REG', self.cmd_PAT_READ_REG)
        self.gcode.register_command(
            'PAT_TEST_UPDATE', self.cmd_PAT_TEST_UPDATE)
        self.gcode.register_command(
            'PAT_RESET', self.cmd_PAT_RESET)
        # XXX ------------------------------------------------
    def handle_ready(self):
        self._setup_extruder_stepper()
        # delay init/update a bit
        self.reactor.register_callback(
            self.start_query, self.reactor.monotonic() + 2.)
    def build_config(self):
        self.mcu.add_config_cmd(
            "command_config_pat9125 oid=%d i2c_oid=%d"
            % (self.oid, self.i2c_oid))
        self.pat9125_query_cmd = self.mcu.lookup_command(
            "command_pat9125_query oid=%c step_oid=%c clock=%u rest_ticks=%u",
            cq=self.cmd_queue)
        self.pat9125_stop_query_cmd = self.mcu.lookup_command(
            "command_pat9125_stop_query oid=%c", cq=self.cmd_queue)
        self.mcu.register_msg(
            self._handle_pat9125_state, "pat9125_state", self.oid)
        self.pat9125_i2c.build_config()
    def _setup_extruder_stepper(self):
        # XXX - see if there is a better way to retreive
        # the extruder stepper's OID than below.  Also may
        # want to get all printer extruders and have an
        # option for which the filament sensor is on
        toolhead = self.printer.lookup_object('toolhead')
        e_stepper = toolhead.get_extruder().stepper
        self.set_stepper(e_stepper)
    def _handle_pat9125_state(self, params):
        if params['flags'] & 0x01 == 0:
            # No ack / comms error
            self.reactor.register_async_callback(
                (lambda e, s=self: s._handle_comms_error()))
            return

        if params['flags'] & FLAG_MOTION:
            # motion detected
            self._pat9125_parse_status(bytearray(params['data']))
        else:
            data = bytearray(params['data'])
            self.pat9125_state['SHUTTER'] = data[3]
            self.pat9125_state['FRAME'] = data[4]

        self.pat9125_state['STEPPER_POS'] = params['epos'] * self.e_step_dist
        # XXX: Logging for debug purposes
        # logging.info(
        #    "PAT9125 Sent_time: %f Receive_time %f"
        #    % (params['#sent_time'], params['#receive_time']))

        self.pat9125_state['MCU_TIME'] = params['#sent_time']
        self.reactor.register_async_callback(
            (lambda e, s=self, p=dict(self.pat9125_state):
                s.watchdog.handle_watchdog_update(e, p)))
    def _handle_comms_error(self):
        self.initialized = False
        self.watchdog.set_comms_error(True)
        logging.info(
            "pat9125: Communication Error encountered, monitoring stopped")
    def _pat9125_parse_status(self, data):
        xl = data[0]
        yl = data[1]
        xyh = data[2]
        dx = check_twos_complement((xl | ((xyh << 4) & 0xF00)), 12)
        dy = check_twos_complement((yl | ((xyh << 8) & 0xF00)), 12)
        if self.invert_axis:
            self.pat9125_state['Y_POS'] -= dy
            self.pat9125_state['X_POS'] -= dx
        else:
            self.pat9125_state['Y_POS'] += dy
            self.pat9125_state['X_POS'] += dx
        self.pat9125_state['SHUTTER'] = data[3]
        self.pat9125_state['FRAME'] = data[4]
    def set_stepper(self, stepper):
        self.e_step_dist = stepper.get_step_dist()
        self.stepper_oid = stepper.mcu_stepper.get_oid()
        get_pos_cmd = self.mcu.lookup_command(
            "stepper_get_position oid=%c", cq=self.cmd_queue)
        self.pat9125_state['STEPPER_POS'] = (
            get_pos_cmd.send_with_response(
                [self.stepper_oid], response='stepper_position',
                response_oid=self.stepper_oid)['pos'] * self.e_step_dist)
    def start_query(self, eventtime):
        if not self.initialized:
            self._pat9125_init()
            if not self.initialized:
                raise self.gcode.error(
                    "PAT9125 not initialized, cannot start query")
        self.pat9125_state['X_POS'] = 0
        self.pat9125_state['Y_POS'] = 0
        self.watchdog.init_xye(self.pat9125_state)
        clock = self.mcu.get_query_slot(self.oid)
        rest_ticks = self.mcu.seconds_to_clock(self.refresh_time)
        self.pat9125_query_cmd.send(
            [self.oid, self.stepper_oid, clock, rest_ticks])
    def stop_query(self):
        # XXX - send query stop command
        self.pat9125_stop_query_cmd.send([self.oid])
    def get_state(self):
        return dict(self.pat9125_state)
    def _pat9125_init(self):
        mcu = self.mcu
        self.initialized = False

        # make sure the mcu timer is stopped
        self.stop_query()

        # Read and verify product ID
        if not self.check_product_id():
            logging.info("Rechecking ID...")
            if not self.check_product_id():
                return

        self.pat9125_i2c.write_register('BANK_SELECTION', 0x00)
        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + .001)
        self.pat9125_i2c.write_register('CONFIG', 0x97, minclock=minclock)
        minclock = mcu.print_time_to_clock(print_time + .002)
        self.pat9125_i2c.write_register('CONFIG', 0x17, minclock=minclock)

        if not (self.pat9125_i2c.write_verify_sequence(PAT9125_INIT1)):
            logging.info("PAT9125: Init 1 failure")
            return

        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + .01)
        self.pat9125_i2c.write_register(
            'BANK_SELECTION', 0x01, minclock=minclock)
        for sequence in PAT9125_INIT2:
            if not self.pat9125_i2c.write_verify_sequence(sequence):
                logging.info("PAT9125: Init 2 failure")
                self.pat9125_i2c.write_register('BANK_SELECTION', 0x00)
                return

        self.pat9125_i2c.write_register('BANK_SELECTION', 0x00)
        if not self.pat9125_i2c.write_verify_sequence(
                [PAT9125_REGS['WP'], 0x00]):
            logging.info("PAT9125: Unable to re-enable write protect")
            return

        if not self.check_product_id():
            return

        self.pat9125_i2c.write_register('RES_X', self.xres)
        self.pat9125_i2c.write_register('RES_Y', self.yres)
        self.initialized = True
        logging.info("PAT9125 Initialization Success")
    def verify_id(self, pid):
        if ((pid[1] << 8) | pid[0]) != PAT9125_PRODUCT_ID:
            logging.info(
                "Product ID Mismatch Expected: %d, Recd: %d"
                % (PAT9125_PRODUCT_ID, ((pid[1] << 8) | pid[0])))
            return False
        return True
    def check_product_id(self):
        pid = self.pat9125_i2c.read_register('PID1', 2)
        return self.verify_id(pid)
    def cmd_PAT_READ_ID(self, params):
        product_id = self.pat9125_i2c.read_register('PID1', 2)
        self.gcode.respond_info(
            "PAT9125 ID: [%#x,%#x]" % (product_id[0], product_id[1]))
    def cmd_PAT_TEST_INIT(self, params):
        logging.info("Attempting Init Test")
        self._pat9125_init()
        if self.initialized:
            msg = "PAT9125 Successfully Initialized"
        else:
            msg = "PAT9125 Initialization failure, check klippy.log"
        self.gcode.respond_info(msg)
    def cmd_PAT_READ_REG(self, params):
        reg = self.gcode.get_str('REG', params)
        reg = int(reg, 16)
        data = self.pat9125_i2c.read_register(reg, 1)[0]
        self.gcode.respond_info(
            "Value at register [%#X]: %d" % (reg, data))
    def cmd_PAT_RESET(self, params):
        self.pat9125_i2c.write_register('BANK_SELECTION', 0x00)
    def cmd_PAT_TEST_UPDATE(self, params):
        test_time = self.gcode.get_int("TIME", params, 5, minval=0)
        force_e = self.gcode.get_str("FORCE_E", params, "false")
        if force_e.lower() == "true":
            force_e = True
        else:
            force_e = False
        if not self.initialized:
            self._pat9125_init()
        self.test_timer = None
        def finish_timer(eventtime):
            self.stop_query()
            self.reactor.unregister_timer(self.test_timer)
            return self.reactor.NEVER
        self.test_timer = self.reactor.register_timer(finish_timer)
        self.start_query(self.reactor.monotonic())
        self.reactor.update_timer(
            self.test_timer, self.reactor.monotonic() + test_time)
        if force_e:
            dist = test_time * 5.
            self.gcode.run_script_from_command(
                "FORCE_MOVE STEPPER=extruder DISTANCE=%f VELOCITY=5"
                % (dist))

class PAT9125_InsertDetect:
    def __init__(self):
        self.callback = None
        self.sum = self.change_counter = 0
        self.enabled = False
    def set_enable(self, enable):
        if enable:
            self.sum = self.change_counter = 0
        self.enabled = enable
    def is_enabled(self):
        return self.enabled
    def set_insert_callback(self, callback):
        self.callback = callback
    def check_insert(self, axis_d, eventtime):
        if axis_d != 0:
            if axis_d > 0:
                self.sum += axis_d
                self.change_counter += 3
            elif self.change_counter > 1:
                self.change_counter -= 2
        elif self.change_counter > 0:
            self.change_counter -= 1

        if not self.change_counter:
            self.sum = 0

        if (self.change_counter >= 12 and self.sum > 20):
            logging.info("PAT9125: Filament Insert detected")
            if self.callback is not None:
                self.callback(eventtime)
            self.sum = self.change_counter = 0
            return True
        return False

class PAT9125_RunoutDetect:
    def __init__(self):
        self.callback = None
        self.error_cnt = 0
    def set_enable(self, enable):
        if enable:
            self.sum = self.error_cnt = 0
        self.enabled = enable
    def is_enabled(self):
        return self.enabled
    def set_insert_callback(self, callback):
        self.callback = callback
    def check_runout(self, axis_d, eventtime):
        pass

TRIGGER_WAIT = 3.
XYE_KEYS = ['X_POS', 'Y_POS', 'STEPPER_POS']

class WatchDog:
    def __init__(self, config, pat9125):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.pat9125 = pat9125
        self.orientation = pat9125.orientation
        self.runout_enabled = False
        self.runout_callback = None
        self.comms_error = False
        self.insert_detect = PAT9125_InsertDetect()
        self.last_event_time = 0.
        self.last_xye = [0, 0, 0]
    def init_xye(self, state):
        for i, key in enumerate(XYE_KEYS):
            self.last_xye[i] = state[key]
    def set_comms_error(self, err):
        self.comms_error = err
        self.runout_enable = False
        self.insert_detect.set_enable(False)
    def set_callbacks(self, runout_cb, insert_cb):
        self.runout_callback = runout_cb
        self.insert_detect.set_insert_callback(insert_cb)
    def set_runout_enable(self, enable):
        if not self.comms_error:
            self.runout_enabled = enable
    def set_insert_enable(self, enable):
        if not self.comms_error:
            self.insert_detect.set_enable(enable)
    def handle_watchdog_update(self, eventtime, pat9125_state):
        curpos = [pat9125_state[key] for key in XYE_KEYS]
        delta = [curpos[i] - self.last_xye[i] for i in range(3)]
        self.last_xye[:] = curpos

        # XXX - logging below for debugging purposes
        # logging.info("PAT9125 status:\n%s" % (str(pat9125_state)))
        # logging.info("Delta: %s" % (str(delta)))

        if eventtime - self.last_event_time < TRIGGER_WAIT:
            # sensor event too close to last event.  bypass
            return
        if self.runout_enabled:
            # XXX Implement runout detection
            # self.last_event_time = eventtime
            pass
        elif self.insert_detect.is_enabled():
            # Ready/Idle, check insert detect
            if self.insert_detect.check_insert(
                    delta[self.orientation], eventtime):
                self.last_event_time = eventtime


def load_config(config):
    return PAT9125(config)
