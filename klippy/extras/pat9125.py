# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import bus
import logging
import threading
import collections
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
        self.printer.add_object('pat9125', self)

        self.stepper_oid = None
        self.pat9125_query_cmd = None
        self.pat9125_stop_query_cmd = None

        self.e_step_dist = 1.
        self.ins_ref_time = config.getfloat(
            "insert_refresh_time", .04, above=0., maxval=1.)
        # TODO: Calculate a sane default for runout_refresh_time using the
        # nozzle size and cross section.We want to be able to poll fast enough
        # to cover .2mm - .5mm of filament movement at the maximum possible
        # extrusion speed. For example, extruding PLA with a .4 nozzle at
        # 185mm/sec needs to refresh approximately .08 seconds to cover .5mm
        # of movement.  A larger nozzle with a bigger cross_section may be able
        # to double that, which  may not be the best candidate for using a
        # PAT9125 to detect filament loss
        self.ro_ref_time = config.getfloat(
            "runout_refresh_time", .07, above=0., maxval=1.)
        self.invert_axis = config.getboolean("invert_axis", False)
        self.oq_enable = int(config.getboolean("oq_enable", False))
        orient_choices = {'X': 0, 'Y': 1}
        self.orientation = config.getchoice(
            "orientation", orient_choices, 'Y')
        self.xres = 0 if self.orientation else 240
        self.yres = 240 if self.orientation else 0
        PAT9125_INIT1[3] = self.xres
        PAT9125_INIT1[5] = self.yres

        self.watchdog = WatchDog(config, self)
        self.set_callbacks = self.watchdog.set_callbacks
        self.set_enable = self.watchdog.set_enable

        self.watch_insert = None
        self.initialized = False
        self.pat9125_state = {
            'STEPPER_POS': 0,
            'X_POS': 0,
            'Y_POS': 0,
            'FRAME': 0,
            'SHUTTER': 0,
            'MOTION': False
        }
    def handle_ready(self):
        self._setup_extruder_stepper()
        # delay init/update so as to not compete with display, tmc, etc
        ins_en = self.watchdog.insert_detect.is_enabled()
        if ins_en or self.watchdog.runout_detect.is_enabled():
            self.reactor.register_callback(
                (lambda e, s=self, i=ins_en: s.start_query(i)),
                self.reactor.monotonic() + 1.)
        else:
            # Neither insert nor runout detection is enabled, just initialize
            self.reactor.register_callback(
                (lambda e, s=self: s._pat9125_init()),
                self.reactor.monotonic() + 1.)
    def build_config(self):
        self.mcu.add_config_cmd(
            "command_config_pat9125 oid=%d i2c_oid=%d oq_enable=%d"
            % (self.oid, self.i2c_oid, self.oq_enable))
        self.pat9125_query_cmd = self.mcu.lookup_command(
            "command_pat9125_query oid=%c step_oid=%c clock=%u rest_ticks=%u",
            cq=self.cmd_queue)
        self.pat9125_stop_query_cmd = self.mcu.lookup_command(
            "command_pat9125_stop_query oid=%c", cq=self.cmd_queue)
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

        self._pat9125_parse_status(
            bytearray(params['data']), params['flags'] & FLAG_MOTION)
        self.pat9125_state['STEPPER_POS'] = params['epos']

        self.reactor.register_async_callback(
            (lambda e, s=self, p=dict(self.pat9125_state):
                s.watchdog.handle_watchdog_update(e, p)))
    def _handle_comms_error(self):
        self.initialized = False
        self.watchdog.set_comms_error(True)
        logging.info(
            "pat9125: Communication Error encountered, monitoring stopped")
    def _pat9125_parse_status(self, data, motion):
        self.pat9125_state['MOTION'] = motion
        if motion:
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
        if self.oq_enable:
            self.pat9125_state['SHUTTER'] = data[3]
            self.pat9125_state['FRAME'] = data[4]
    def set_stepper(self, stepper):
        self.e_step_dist = stepper.get_step_dist()
        self.stepper_oid = stepper.mcu_stepper.get_oid()
        self.watchdog.runout_detect.set_move_limits(self.e_step_dist)
    def start_query(self, ins_enable):
        if self.watch_insert is not None and self.watch_insert is ins_enable:
            # query already started with the correct refresh time
            return
        if not self.initialized:
            self._pat9125_init()
            if not self.initialized:
                raise self.gcode.error(
                    "PAT9125 not initialized, cannot start query")
        clock = self.mcu.get_query_slot(self.oid)
        # Determine rest ticks
        self.watch_insert = ins_enable
        refresh_time = self.ins_ref_time if ins_enable else self.ro_ref_time
        rest_ticks = self.mcu.seconds_to_clock(refresh_time)
        self.pat9125_query_cmd.send(
            [self.oid, self.stepper_oid, clock, rest_ticks])
        self.watchdog.need_xye_init = True
        logging.info(
            "PAT9125: Polling activated, sampling every %.4f ms"
            % (refresh_time))
    def stop_query(self):
        self.pat9125_stop_query_cmd.send([self.oid])
        self.watch_insert = None
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

        self.pat9125_state['X_POS'] = 0
        self.pat9125_state['Y_POS'] = 0
        self.watchdog.need_xye_init = True
        self.mcu.register_msg(
            self._handle_pat9125_state, "pat9125_state", self.oid)

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
    def query_status(self):
        state = self.get_state()
        if state['MOTION']:
            msg = "pat9125 sensor: Motion Detected"
        else:
            msg = "pat9125 sensor: Motion not Detected"
        msg += "\nCurrent Position: X=%d Y=%d STEPPER=%.6f" % (
            state['X_POS'], state['Y_POS'],
            state['STEPPER_POS'] * self.e_step_dist)
        if self.oq_enable:
            msg += "\nAverage Brightness: %d Laser Shutter Time: %d" % (
                state['FRAME'], state['SHUTTER'])
        return msg

class PAT9125_InsertDetect:
    def __init__(self, config):
        self.reactor = config.get_printer().get_reactor()
        self._counter_max = config.getint(
            'max_insert_counter', 12, minval=1)
        self._sum_max = config.getint(
            'max_insert_sum', 20, minval=1)
        self.callback = None
        self.sum = self.change_counter = 0
        self.enabled = False
    def set_enable(self, enable):
        if enable:
            self.sum = self.change_counter = 0
        self.enabled = enable
    def is_enabled(self):
        return self.enabled
    def set_callback(self, callback):
        self.callback = callback
    def check_insert(self, axis_d):
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

        if (self.change_counter >= self._counter_max and
                self.sum > self._sum_max):
            logging.info("PAT9125: Filament Insert detected")
            self.sum = self.change_counter = 0
            if self.callback is not None:
                self.reactor.register_callback(self.callback)
            return True
        return False

class PAT9125_RunoutDetect:
    def __init__(self, config):
        self.reactor = config.get_printer().get_reactor()
        self._max_error = config.getint(
            'max_runout_errors', 17, minval=1)
        self._blind_move_max = config.getfloat(
            'max_blind_movement', 2., above=0.)
        self._blind_error_distance = config.getfloat(
            'blind_error_distance', .3, above=0.)
        self.callback = None
        self.total_error_cnt = 0
        self.blind_error_cnt = 0
        self.prev_d = 0
        self.blind_steps = 0
        self.max_blind_steps = 0
        self.min_error_steps = 0
    def set_enable(self, enable):
        if enable:
            self.total_error_cnt = self.blind_error_cnt = 0
            self.blind_steps = self.prev_d = 0
        self.enabled = enable
    def set_move_limits(self, step_distance):
        self.max_blind_steps = int(self._blind_move_max / step_distance + .5)
        self.min_error_steps = int(
            self._blind_error_distance / step_distance + .5)
    def is_enabled(self):
        return self.enabled
    def set_callback(self, callback):
        self.callback = callback
    def check_runout(self, axis_d, stepper_d):
        if stepper_d > 0:
            # extruder is moving in the positive direction
            if axis_d < -5:
                # Small deltas that really don't indicate
                # significant travel in the negative direction: ie
                # a jam.  A small amount of negative movement will
                # just be interpreted as a pure blind movement.
                logging.info(
                    "PAT9125: Negative motion detected,"
                    " Axis Delta: %d Stepper Delta %d" %
                    (axis_d, stepper_d))
                self.blind_steps += stepper_d
                if self.total_error_cnt:
                    self.total_error_cnt += 2
                else:
                    self.total_error_cnt += 1
            elif axis_d > 0:
                # positive movement, we are good
                self.blind_steps = self.blind_error_cnt = 0
                if self.total_error_cnt:
                    self.total_error_cnt -= 1
            else:
                # No movement detected, we are stepping blind.
                # TODO: Not sure this is the best solution.  I
                # may need to come up with a completely unique
                # solution to reliably determine when a
                # runout occurs in as short of time as possible
                self.blind_steps += stepper_d
                cur_err = self.blind_steps / (self.blind_error_cnt + 1)
                if ((self.prev_d <= 0 or self.total_error_cnt) and
                        cur_err > self.min_error_steps):
                    self.total_error_cnt += 1
                    self.blind_error_cnt += 1

        self.prev_d = axis_d
        if (self.total_error_cnt > self._max_error or
                self.blind_steps > self.max_blind_steps):
            logging.info(
                "PAT9125 Runout Detected - Error Count: %d Blind Steps: %d" %
                (self.total_error_cnt, self.blind_steps))
            self.total_error_cnt = self.blind_error_cnt = 0
            self.blind_steps = self.prev_d = 0
            if self.callback is not None:
                self.reactor.register_callback(self.callback)
            return True
        return False

XYE_KEYS = ['X_POS', 'Y_POS', 'STEPPER_POS']

class WatchDog:
    def __init__(self, config, pat9125):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.pat9125 = pat9125
        self.oq_enable = pat9125.oq_enable
        self.orientation = pat9125.orientation
        self.comms_error = False
        self.insert_detect = PAT9125_InsertDetect(config)
        self.runout_detect = PAT9125_RunoutDetect(config)
        self.insert_delay = config.getfloat('insert_event_delay', 3., above=0.)
        # XXX - ideally the runout_delay should be above the toolhead's
        # max lookahead buffer time to make sure additional runouts
        # aren't triggered before stopping
        self.runout_delay = config.getfloat(
            'runout_event_delay', self.insert_delay, above=0.)
        self.need_xye_init = True
        self.last_event_time = 0.
        self.last_xye = [0, 0, 0]
        self.oq_deque = collections.deque(maxlen=20)
    def set_comms_error(self, err):
        self.comms_error = err
        self.runout_detect.set_enable(False)
        self.insert_detect.set_enable(False)
    def set_callbacks(self, runout_cb, insert_cb):
        self.runout_detect.set_callback(runout_cb)
        self.insert_detect.set_callback(insert_cb)
    def set_enable(self, insert, runout):
        if insert and runout:
            # only one can be enabled at a time
            insert = False
        if not self.comms_error:
            self.insert_detect.set_enable(insert)
            self.runout_detect.set_enable(runout)
            if self.pat9125.initialized:
                if not insert and not runout:
                    self.pat9125.stop_query()
                else:
                    self.pat9125.start_query(insert)
    def log_oq_results(self):
        if self.oq_enable:
            logging.info(
                "PAT9125: Last 20 OQ results (shutter, frame) %s" %
                str(list(self.oq_deque)))
    def handle_watchdog_update(self, eventtime, state):
        curpos = [state[key] for key in XYE_KEYS]
        delta = [curpos[i] - self.last_xye[i] for i in range(3)]
        self.oq_deque.append((state['SHUTTER'], state['FRAME']))
        self.last_xye[:] = curpos
        if self.need_xye_init:
            self.need_xye_init = False
            return

        # XXX - logging below for debugging purposes
        # logging.info("PAT9125 status:\n%s" % (str(pat9125_state)))
        # logging.info("Delta: %s" % (str(delta)))

        if self.runout_detect.is_enabled():
            if eventtime - self.last_event_time < self.runout_delay:
                return
            if self.runout_detect.check_runout(
                    delta[self.orientation], delta[2]):
                self.last_event_time = eventtime
                self.log_oq_results()
        elif self.insert_detect.is_enabled():
            if eventtime - self.last_event_time < self.insert_delay:
                return
            if self.insert_detect.check_insert(
                    delta[self.orientation]):
                self.last_event_time = eventtime
                self.log_oq_results()
