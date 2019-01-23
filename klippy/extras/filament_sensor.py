# Generic Filament Sensor Module
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

SENSOR_TYPES = {'switch': 0, 'pat9125': 1}

class FilamentSensor:
    def __init__(self, config):
        self.name = config.get_name().split()[1]
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.display = self.printer.try_load_module(config, 'display')
        sensor_type = config.getchoice('sensor_type', SENSOR_TYPES, "switch")
        if sensor_type == 0:
            # Basic switch sensor
            self.sensor = SwitchSensor(config)
        elif sensor_type == 1:
            # PAT9125 I2C Laser Sensor.  Only one instance is allowed
            import pat9125
            self.sensor = pat9125.PAT9125(config)
        self.autoload_on = config.getboolean('autoload_on', False)
        self.set_enable = self.sensor.set_enable
        self.set_enable(self.autoload_on, False)
        self.sensor.set_callbacks(
            self.runout_event_handler, self.autoload_event_handler)
        self.test_mode = config.get('test_mode', False)
        self.runout_gcode = config.get('runout_gcode', None)
        self.autoload_gcode = config.get('autoload_gcode', None)
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
            "QUERY_FILAMENT_SENSOR", "SENSOR", self.name,
            self.cmd_QUERY_FILAMENT_SENSOR,
            desc=self.cmd_QUERY_FILAMENT_SENSOR_help)
    def handle_ready(self):
        self.v_sd = self.printer.lookup_object('virtual_sdcard', None)
        self.toolhead = self.printer.lookup_object('toolhead')
    def get_sensor(self):
        return self.sensor
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
            self.set_enable(False, True)
        else:
            self.set_enable(self.autoload_on, False)
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
        self.toolhead.capture_position()
        if self.runout_gcode is not None:
            self.gcode.run_script_from_command(self.runout_gcode)
    def autoload_event_handler(self, eventtime):
        self.notify("Filament Autoload Event Detected", d_timeout=5)
        if self.test_mode:
            return
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
        # reset autoload state in case this was user intiated
        if not self.autoload_on:
            self.update_print_status(self.print_status)
    def notify(self, msg, with_logging=True, d_timeout=None):
        if self.display is not None:
            self.display.set_message(msg, d_timeout)
        self.gcode.respond_info(msg)
        if with_logging:
            logging.info("filament_sensor: " + msg)
    cmd_QUERY_FILAMENT_SENSOR_help = "Querey the status of the Filament Sensor"
    def cmd_QUERY_FILAMENT_SENSOR(self, params):
        msg = self.sensor.query_status()
        self.gcode.respond_info(msg)
    cmd_AUTOLOAD_FILAMENT_help = "Allows user initated autoloading"
    def cmd_AUTOLOAD_FILAMENT(self, params):
        if self.print_status == "printing":
            self.gcode.respond_info("Cannot autoload filament while printing")
        elif not self.autoload_on:
            # autoload isn't on, go ahead and enable
            self.sensor.set_enable(True, False)


SETTLE_TIME = .1

class SwitchSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.buttons = self.printer.try_load_module(config, 'buttons')
        switch_pin = config.get('switch_pin')
        self.buttons.register_buttons([switch_pin], self.switch_event_handler)
        self.insert_delay = config.getfloat('insert_event_delay', 3., above=0.)
        # XXX - ideally the runout_delay should be above the toolhead's
        # max lookahead buffer time to make sure additional runouts
        # aren't triggered before stopping
        self.runout_delay = config.getfloat(
            'runout_event_delay', self.insert_delay, above=0.)
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
    def set_enable(self, insert, runout):
        if insert and runout:
            # both cannot be enabled
            insert = False
        self.insert_enabled = insert
        self.runout_enabled = runout
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
        elif self.event_button_state:
            # button pushed, check if insert callback should happen
            if self.insert_enabled:
                if eventtime - self.last_cb_event_time < self.insert_delay:
                    return self.reactor.NEVER
                self.last_cb_event_time = eventtime
                if self.insert_cb is not None:
                    self.reactor.register_callback(self.insert_cb)
        elif self.runout_enabled:
            # Filament runout detected
            if eventtime - self.last_cb_event_time < self.runout_delay:
                return self.reactor.NEVER
            self.last_cb_event_time = eventtime
            if self.runout_cb is not None:
                self.reactor.register_callback(self.runout_cb)
        self.event_button_state = None
        return self.reactor.NEVER
    def query_status(self):
        if self.last_button_state:
            return "Switch Sensor: Switch is closed, filament detected"
        else:
            return "Switch Sensor: Switch is open, filament not detected"

def load_config_prefix(config):
    return FilamentSensor(config)
