MCDMA loopback PCI device
-------------------------

QEMU provides an emulated PCI MCDMA device named ``mcdma``.

The device currently models queue control/status registers, descriptor
processing, and loopback between TX and RX queues.

Basic usage
~~~~~~~~~~~

.. parsed-literal::

   |qemu_system_x86| -device mcdma

Device properties
~~~~~~~~~~~~~~~~~

``num_queues``
  Number of RX/TX queue pairs exposed by the device. The default value is 1.

Example with four queue pairs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. parsed-literal::

   |qemu_system_x86| -device mcdma,num_queues=4

Notes
~~~~~

- The MCDMA device is a PCI device, so normal PCI placement options
  (for example ``addr=0x4``) can be used with ``-device`` as needed.
- The guest driver is expected to program queue rings through the MCDMA
  BAR/register interface.
