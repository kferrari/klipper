# Generic Filament Sensor Module
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

SENSOR_TYPES = ["switch", "pat9125"]

class FilamentSensor:
    def __init__(self, config):
        self.name = config.get_name().split()[1]
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.display = self.printer.try_load_module(config, 'display')
        sensor_type = config.get('sensor_type', "switch").lower()
        if sensor_type not in SENSOR_TYPES:
            raise config.error(
                "Filament Sensor: Unsupported sensor type: %s" % sensor_type)
        if sensor_type == "switch":
            self.sensor = SwitchSensor(config)
        else:
            self.sensor = self.printer.try_load_module(config, sensor_type)
            if self.sensor is None:
                raise config.error(
                    "Unable to load module for sensor: %s" % sensor_type)
        self.set_insert_enable = self.sensor.set_insert_enable
        self.set_runout_enable = self.sensor.set_runout_enable
        self.autoload_on = config.getboolean('autoload_on', False)
        self.set_insert_enable(self.autoload_on)
        self.sensor.set_callbacks(
            self.runout_event_handler, self.autoload_event_handler)
        self.test_mode = config.get('test_mode', False)
        self.runout_gcode = config.get('runout_gcode', None)
        self.autoload_gcode = config.get('autoload_gcode', None)
        self.paused_position = None
        self.monitor_state = True
        self.print_status = "idle"
        for status in ["idle", "ready", "printing"]:
            self.printer.register_event_handler(
                "idle_timeout:%s" % (status),
                (lambda e, s=self, st=status: s.update_print_status(st)))
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.gcode.register_mux_command(
            "AUTOLOAD_FILAMENT", "SENSOR", self.name,
            self.cmd_AUTOLOAD_FILAMENT,
            desc=self.cmd_AUTOLOAD_FILAMENT_help)
        self.gcode.register_mux_command(
            "RESUME_PRINT", "SENSOR", self.name,
            self.cmd_RESUME_PRINT)
    def handle_ready(self):
        self.v_sd = self.printer.lookup_object('virtual_sdcard', None)
        self.toolhead = self.printer.lookup_object('toolhead')
    def set_callbacks(self, runout_cb=None, detected_cb=None, monitor=True):
        # Allow external modules to recieve filament status notifications
        if runout_cb is None:
            runout_cb = self.runout_event_handler
        if detected_cb is None:
            detected_cb = self.autoload_event_handler
        # When monitor is set to true, the filament_sensor module will monitor
        # printer status.  If Klipper is printing, the runout detection
        # is enbabled, insert detction disabled, and vice versa
        # when not printing.  External modules may set this to false
        # to enable/disable detection as they need.
        self.monitor_state = monitor
        self.sensor.set_callbacks(runout_cb, detected_cb)
    def update_print_status(self, status):
        logging.info(
            "filament_sensor: print status changed to: %s" % (status))
        self.print_status = status
        if not self.monitor_state:
            return
        if status == "printing":
            self.set_runout_enable(True)
            self.set_insert_enable(False)
        else:
            self.set_runout_enable(False)
            self.set_insert_enable(self.autoload_on)
    def runout_event_handler(self, eventtime):
        self.notify("Filament Runout Event Detected", d_timeout=5)
        if self.test_mode:
            return
        # TODO: we may need a lower level pause implementation.  The issue
        # is that with a full lookahead queue its possible
        # that filament will move past the extruder gears before
        # the queue is flushed and we get an actual pause
        if self.v_sd is not None and self.v_sd.is_active():
            # Printing from virtual sd, run pause command
            self.v_sd.cmd_M25({})
        else:
            self.gcode.respond_info("action:pause")
        self.toolhead.wait_moves()
        self.paused_position = self.toolhead.get_position()
        if self.runout_gcode is not None:
            self.gcode.run_script_from_command(self.runout_gcode)
    def autoload_event_handler(self, eventtime):
        self.notify("Filament Autoload Event Detected", d_timeout=5)
        if self.test_mode:
            return
        # reset autoload state in case this was user intiated
        self.set_insert_enable(self.autoload_on)
        heater = self.toolhead.get_extruder().get_heater()
        if heater.can_extrude:
            self.notify("Autoloading Filament...", with_logging=False)
            if self.autoload_gcode is not None:
                self.gcode.run_script_from_command(self.autoload_gcode)
                self.toolhead.wait_moves()
                self.notify(
                    "Autoload Complete", with_logging=False, d_timeout=5)
        else:
            self.notify(
                "Must pre-heat extruder before autoloading", d_timeout=5)
    def notify(self, msg, with_logging=True, d_timeout=None):
        if self.display is not None:
            self.display.set_message(msg, d_timeout)
        self.gcode.respond_info(msg)
        if with_logging:
            logging.info("filament_sensor: " + msg)
    cmd_AUTOLOAD_FILAMENT_help = "Allows user initated autoloading"
    def cmd_AUTOLOAD_FILAMENT(self, params):
        if self.print_status == "printing":
            self.gcode.respond_info("Cannot autoload filament while printing")
        else:
            self.sensor.set_insert_enable(True)
    def cmd_RESUME_PRINT(self, params):
        speed = self.gcode.get_float('SPEED', params, 50.)
        if self.paused_position is not None:
            curpos = self.toolhead.get_position()
            self.paused_position[3] = curpos[3]
            self.toolhead.move(self.paused_position, speed)
            self.toolhead.wait_moves()
            self.gcode.reset_last_position()
            self.paused_position = None
            eventtime = self.printer.get_reactor().monotonic()
            if self.v_sd is not None and self.v_sd.is_active():
                # Printing from virtual sd, run pause command
                self.v_sd.cmd_M24({})
            else:
                self.gcode.respond_info("action:resume")


TRIGGER_WAIT = 3.
SETTLE_TIME = .1

class SwitchSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.buttons = self.printer.try_load_module(config, 'buttons')
        switch_pin = config.get('switch_pin')
        self.buttons.register_buttons([switch_pin], self.switch_event_handler)
        self.insert_enabled = False
        self.runout_enabled = False
        self.runout_cb = self.insert_cb = None
        self.event_button_state = None
        self.last_button_state = False
        self.last_cb_event_time = 0.
        self.switch_event_timer = self.reactor.register_timer(
            self.event_exec_timer)
    def set_callbacks(self, runout_cb, insert_cb):
        self.runout_cb = runout_cb
        self.insert_cb = insert_cb
    def set_runout_enable(self, enable):
        self.runout_enabled = enable
    def set_insert_enable(self, enable):
        self.insert_enabled = enable
    def switch_event_handler(self, eventtime, state):
        self.last_button_state = state
        if self.event_button_state is not None:
            # currently waiting on for an event to exectute
            logging.info(
                "SwitchSensor: Received swtich event during timer")
            return
        self.event_button_state = state
        self.reactor.update_timer(
            self.switch_event_timer, eventtime + SETTLE_TIME)
    def event_exec_timer(self, eventtime):
        if self.event_button_state ^ self.last_button_state:
            # button state has changed since timer was executed
            logging.info("SwitchSensor: False positive detected")
        elif eventtime - self.last_cb_event_time < TRIGGER_WAIT:
            logging.info("SwitchSensor: Sensor event too soon")
        elif self.event_button_state:
            # button pushed, check if insert callback should happen
            if self.insert_enabled:
                self.last_cb_event_time = eventtime
                if self.insert_cb is not None:
                    self.insert_cb(eventtime)
        elif self.runout_enabled:
            # Filament runout detected
            self.last_cb_event_time = eventtime
            if self.runout_cb is not None:
                self.runout_cb(eventtime)
        self.event_button_state = None
        return self.reactor.NEVER

def load_config_prefix(config):
    return FilamentSensor(config)
