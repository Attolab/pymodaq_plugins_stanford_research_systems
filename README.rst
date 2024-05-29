pymodaq_plugins_stanford_research_systems
#########################################

.. the following must be adapted to your developed package, links to pypi, github  description...

.. image:: https://img.shields.io/pypi/v/pymodaq_plugins_stanford_research_systems.svg
   :target: https://pypi.org/project/pymodaq_plugins_stanford_research_systems/
   :alt: Latest Version

.. image:: https://readthedocs.org/projects/pymodaq/badge/?version=latest
   :target: https://pymodaq.readthedocs.io/en/stable/?badge=latest
   :alt: Documentation Status

.. image:: https://github.com/PyMoDAQ/pymodaq_plugins_stanford_research_systems/workflows/Upload%20Python%20Package/badge.svg
   :target: https://github.com/PyMoDAQ/pymodaq_plugins_stanford_research_systems
   :alt: Publication Status

.. image:: https://github.com/PyMoDAQ/pymodaq_plugins_stanford_research_systems/actions/workflows/Test.yml/badge.svg
    :target: https://github.com/PyMoDAQ/pymodaq_plugins_stanford_research_systems/actions/workflows/Test.yml


Set of instruments from the Stanford Research Systems brand

Authors
=======

* Sebastien J. Weber  (sebastien.weber@cemes.fr)



Instruments
===========

Below is the list of instruments included in this plugin

Actuators
++++++++
* **ChopperSR542**: Precision Optical Chopper SR542 using the srsinst.sr542 package provided by Stanford Research Systems.
Most chopper settings are available in the parameter tree, and the chopper phase controlled by the move commands.


Viewer0D
++++++++

* **LockinSR830Legacy**: LockIn Amplifier SR830 using direct VISA SPI commands (no more maintained)
* **LockinSR830**: LockIn Amplifier SR830 using the pymeasure SR830 driver (preferred)


Viewer1D
++++++++

* **LockinSR830**: LockIn Amplifier SR830 using the pymeasure SR830 driver and using the internal buffer memory. Get the
  two channels outputs. Should be fast but is in fact quite slow...



Installation instructions
=========================

* PyMoDAQ version > 4.0.8
* VISA backend if connected using GPIB
* pymeasure version >= 0.14.0 needed (if not yet available install from its github repository)
* For SR542 Chopper, the USB-to-serial driver might need to be installed so that the controller appears a COM port (available at https://ftdichip.com/drivers/vcp-drivers/)
