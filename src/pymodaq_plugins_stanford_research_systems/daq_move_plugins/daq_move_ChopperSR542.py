from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main, DataActuatorType, \
    DataActuator  # common set of parameters for all actuators
from pymodaq.utils.daq_utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq.utils.parameter import Parameter
from srsinst.sr542 import SR542
import serial.tools.list_ports

comports = [port for port,_,_ in sorted(serial.tools.list_ports.comports())]


class DAQ_Move_ChopperSR542(DAQ_Move_base):
    """ Instrument plugin class for an actuator.

    This object inherits all functionalities to communicate with PyMoDAQ’s DAQ_Move module through inheritance via
    DAQ_Move_base. It makes a bridge between the DAQ_Move module and the Python wrapper of a particular instrument.

        * Tested on Windows 10, python 3.9, pymodaq 4.2.0
        * Installation instructions: if the device is not properly detected as COM port, install virtual COM ports driver
        from https://ftdichip.com/drivers/vcp-drivers/

    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library.

    """
    _controller_units = 'deg'
    is_multiaxes = False
    _axis_names = ['Phase']
    _epsilon = 0.1
    data_actuator_type = DataActuatorType['DataActuator']

    params = [
                 {'title': 'COM port:', 'name': 'com_port', 'type': 'list', 'limits': comports, 'value': comports[0]},
                 {'title': 'Source:', 'name': 'source', 'type': 'list',
                  'limits': ['internal', 'vco', 'line', 'external'], 'value': 'internal'},
                 {'title': 'Edge:', 'name': 'edge', 'type': 'list',
                  'limits': ['rise', 'fall', 'sine'], 'value': 'rise'},
                 {'title': 'Internal Frequency:', 'name': 'internal_freq', 'type': 'float', 'value': 100.0},
                 {'title': 'Multiplier n:', 'name': 'n', 'type': 'int', 'min': 1, 'value': 1},
                 {'title': 'Multiplier m:', 'name': 'm', 'type': 'int', 'min': 1, 'value': 1},
                 {'title': 'Control:', 'name': 'control', 'type': 'list', 'limits': ['shaft', 'inner', 'outer'], 'value': 'outer'},
                 {'title': 'Run/Stop', 'name': 'run', 'type': 'bool_push', 'value': False},
             ] + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    def ini_attributes(self):
        self.controller: SR542 = None


    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        pos = DataActuator(data=self.controller.config.phase)  # when writing your own plugin replace this line
        pos = self.get_position_with_scaling(pos)
        return pos

    def close(self):
        """Terminate the communication protocol"""
        self.controller.disconnect()

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.name() == "source":
            self.controller.config.source = param.value()
            if param.value() == 'external':
                self.settings.child('edge').show()
            else:
                self.settings.child('edge').hide()

            if param.value() == 'internal':
                self.settings.child('internal_freq').show()
            else:
                self.settings.child('internal_freq').hide()

        elif param.name() == "edge":
            self.controller.config.sync_edge = param.value()

        elif param.name() == "internal_freq":
            self.controller.config.internal_freq = param.value()

        elif param.name() == "control":
            self.controller.config.control_target = param.value()

        elif param.name() == "run":
            if param.value():
                self.controller.operate.run()
            else:
                self.controller.operate.stop()

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """

        self.controller = self.ini_stage_init(old_controller=controller,
                                              new_controller=SR542('serial', self.settings['com_port'], '115200'))

        info = "Chopper initialized"
        initialized = self.controller.is_connected()

        for param in self.settings.children():
            self.commit_settings(param)

        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  # if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        self.controller.config.phase = value.value()
        # self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)
        self.controller.config.phase = self.target_value.value()
        # self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))

    def move_home(self):
        """Call the reference method of the controller"""
        pass

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""

        self.controller.operate.stop()
        self.settings['run'] = False


if __name__ == '__main__':
    main(__file__)