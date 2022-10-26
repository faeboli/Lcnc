MMIO_INST
=========

Register Listing for MMIO_INST
------------------------------

+------------------------------------------------------+-------------------------------------------+
| Register                                             | Address                                   |
+======================================================+===========================================+
| :ref:`MMIO_INST_VELOCITY_0 <MMIO_INST_VELOCITY_0>`   | :ref:`0x00000000 <MMIO_INST_VELOCITY_0>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_VELOCITY_1 <MMIO_INST_VELOCITY_1>`   | :ref:`0x00000004 <MMIO_INST_VELOCITY_1>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_VELOCITY_2 <MMIO_INST_VELOCITY_2>`   | :ref:`0x00000008 <MMIO_INST_VELOCITY_2>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_VELOCITY_3 <MMIO_INST_VELOCITY_3>`   | :ref:`0x0000000c <MMIO_INST_VELOCITY_3>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_VELOCITY_4 <MMIO_INST_VELOCITY_4>`   | :ref:`0x00000010 <MMIO_INST_VELOCITY_4>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_VELOCITY_5 <MMIO_INST_VELOCITY_5>`   | :ref:`0x00000014 <MMIO_INST_VELOCITY_5>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_STEP_RES_EN <MMIO_INST_STEP_RES_EN>` | :ref:`0x00000018 <MMIO_INST_STEP_RES_EN>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_STEPDIRINV <MMIO_INST_STEPDIRINV>`   | :ref:`0x0000001c <MMIO_INST_STEPDIRINV>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_STEPTIMES <MMIO_INST_STEPTIMES>`     | :ref:`0x00000020 <MMIO_INST_STEPTIMES>`   |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_GPIOS_OUT <MMIO_INST_GPIOS_OUT>`     | :ref:`0x00000024 <MMIO_INST_GPIOS_OUT>`   |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_PWM_0 <MMIO_INST_PWM_0>`             | :ref:`0x00000028 <MMIO_INST_PWM_0>`       |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_PWM_1 <MMIO_INST_PWM_1>`             | :ref:`0x0000002c <MMIO_INST_PWM_1>`       |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_ENC_RES_EN <MMIO_INST_ENC_RES_EN>`   | :ref:`0x00000030 <MMIO_INST_ENC_RES_EN>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_RES_ST_REG <MMIO_INST_RES_ST_REG>`   | :ref:`0x00000034 <MMIO_INST_RES_ST_REG>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_SG_COUNT_0 <MMIO_INST_SG_COUNT_0>`   | :ref:`0x00000038 <MMIO_INST_SG_COUNT_0>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_SG_COUNT_1 <MMIO_INST_SG_COUNT_1>`   | :ref:`0x0000003c <MMIO_INST_SG_COUNT_1>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_SG_COUNT_2 <MMIO_INST_SG_COUNT_2>`   | :ref:`0x00000040 <MMIO_INST_SG_COUNT_2>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_SG_COUNT_3 <MMIO_INST_SG_COUNT_3>`   | :ref:`0x00000044 <MMIO_INST_SG_COUNT_3>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_SG_COUNT_4 <MMIO_INST_SG_COUNT_4>`   | :ref:`0x00000048 <MMIO_INST_SG_COUNT_4>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_SG_COUNT_5 <MMIO_INST_SG_COUNT_5>`   | :ref:`0x0000004c <MMIO_INST_SG_COUNT_5>`  |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_WALLCLOCK <MMIO_INST_WALLCLOCK>`     | :ref:`0x00000050 <MMIO_INST_WALLCLOCK>`   |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_GPIOS_IN <MMIO_INST_GPIOS_IN>`       | :ref:`0x00000054 <MMIO_INST_GPIOS_IN>`    |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_ENC_COUNT_0 <MMIO_INST_ENC_COUNT_0>` | :ref:`0x00000058 <MMIO_INST_ENC_COUNT_0>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_ENC_COUNT_1 <MMIO_INST_ENC_COUNT_1>` | :ref:`0x0000005c <MMIO_INST_ENC_COUNT_1>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_ENC_COUNT_2 <MMIO_INST_ENC_COUNT_2>` | :ref:`0x00000060 <MMIO_INST_ENC_COUNT_2>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_ENC_COUNT_3 <MMIO_INST_ENC_COUNT_3>` | :ref:`0x00000064 <MMIO_INST_ENC_COUNT_3>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_ENC_COUNT_4 <MMIO_INST_ENC_COUNT_4>` | :ref:`0x00000068 <MMIO_INST_ENC_COUNT_4>` |
+------------------------------------------------------+-------------------------------------------+
| :ref:`MMIO_INST_ENC_COUNT_5 <MMIO_INST_ENC_COUNT_5>` | :ref:`0x0000006c <MMIO_INST_ENC_COUNT_5>` |
+------------------------------------------------------+-------------------------------------------+

MMIO_INST_VELOCITY_0
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x0 = 0x00000000`

    Stepgen velocity

    .. wavedrom::
        :caption: MMIO_INST_VELOCITY_0

        {
            "reg": [
                {"name": "velocity_0[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_VELOCITY_1
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x4 = 0x00000004`

    Stepgen velocity

    .. wavedrom::
        :caption: MMIO_INST_VELOCITY_1

        {
            "reg": [
                {"name": "velocity_1[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_VELOCITY_2
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x8 = 0x00000008`

    Stepgen velocity

    .. wavedrom::
        :caption: MMIO_INST_VELOCITY_2

        {
            "reg": [
                {"name": "velocity_2[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_VELOCITY_3
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0xc = 0x0000000c`

    Stepgen velocity

    .. wavedrom::
        :caption: MMIO_INST_VELOCITY_3

        {
            "reg": [
                {"name": "velocity_3[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_VELOCITY_4
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x10 = 0x00000010`

    Stepgen velocity

    .. wavedrom::
        :caption: MMIO_INST_VELOCITY_4

        {
            "reg": [
                {"name": "velocity_4[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_VELOCITY_5
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x14 = 0x00000014`

    Stepgen velocity

    .. wavedrom::
        :caption: MMIO_INST_VELOCITY_5

        {
            "reg": [
                {"name": "velocity_5[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_STEP_RES_EN
^^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x18 = 0x00000018`

    Stepgen Enable and Reset flags

    .. wavedrom::
        :caption: MMIO_INST_STEP_RES_EN

        {
            "reg": [
                {"name": "sgreset",  "bits": 16},
                {"name": "sgenable",  "bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+----------+-------------+
| Field   | Name     | Description |
+=========+==========+=============+
| [15:0]  | SGRESET  | Reset       |
+---------+----------+-------------+
| [31:16] | SGENABLE | Enable      |
+---------+----------+-------------+

MMIO_INST_STEPDIRINV
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x1c = 0x0000001c`

    Stepgen Dir and Step inversion

    .. wavedrom::
        :caption: MMIO_INST_STEPDIRINV

        {
            "reg": [
                {"name": "dir_inv",  "bits": 16},
                {"name": "step_inv",  "bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+----------+--------------------+
| Field   | Name     | Description        |
+=========+==========+====================+
| [15:0]  | DIR_INV  | Dir Pin Inversion  |
+---------+----------+--------------------+
| [31:16] | STEP_INV | Step Pin Inversion |
+---------+----------+--------------------+

MMIO_INST_STEPTIMES
^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x20 = 0x00000020`

    Stepgen steptime

    .. wavedrom::
        :caption: MMIO_INST_STEPTIMES

        {
            "reg": [
                {"name": "dir_setup",  "bits": 14},
                {"name": "dir_width",  "bits": 9},
                {"name": "step_width",  "bits": 9}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+------------+------------------------+
| Field   | Name       | Description            |
+=========+============+========================+
| [13:0]  | DIR_SETUP  | Dir Pin Setup time     |
+---------+------------+------------------------+
| [22:14] | DIR_WIDTH  | Dir Pin Minimum width  |
+---------+------------+------------------------+
| [31:23] | STEP_WIDTH | Step Pin Minimum width |
+---------+------------+------------------------+

MMIO_INST_GPIOS_OUT
^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x24 = 0x00000024`

    gpios out

    .. wavedrom::
        :caption: MMIO_INST_GPIOS_OUT

        {
            "reg": [
                {"name": "gpios_out[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_PWM_0
^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x28 = 0x00000028`

    PWM width and period

    .. wavedrom::
        :caption: MMIO_INST_PWM_0

        {
            "reg": [
                {"name": "width",  "bits": 16},
                {"name": "period",  "bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+--------+-------------+
| Field   | Name   | Description |
+=========+========+=============+
| [15:0]  | WIDTH  | PWM Width   |
+---------+--------+-------------+
| [31:16] | PERIOD | PWM Period  |
+---------+--------+-------------+

MMIO_INST_PWM_1
^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x2c = 0x0000002c`

    PWM width and period

    .. wavedrom::
        :caption: MMIO_INST_PWM_1

        {
            "reg": [
                {"name": "width",  "bits": 16},
                {"name": "period",  "bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+--------+-------------+
| Field   | Name   | Description |
+=========+========+=============+
| [15:0]  | WIDTH  | PWM Width   |
+---------+--------+-------------+
| [31:16] | PERIOD | PWM Period  |
+---------+--------+-------------+

MMIO_INST_ENC_RES_EN
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x30 = 0x00000030`

    Encoder enable and reset flags

    .. wavedrom::
        :caption: MMIO_INST_ENC_RES_EN

        {
            "reg": [
                {"name": "reset",  "bits": 16},
                {"name": "enable",  "bits": 16}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+--------+-------------+
| Field   | Name   | Description |
+=========+========+=============+
| [15:0]  | RESET  | Reset       |
+---------+--------+-------------+
| [31:16] | ENABLE | Enable      |
+---------+--------+-------------+

MMIO_INST_RES_ST_REG
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x34 = 0x00000034`

    Reset and status register

    .. wavedrom::
        :caption: MMIO_INST_RES_ST_REG

        {
            "reg": [
                {"bits": 10},
                {"name": "watchdog",  "bits": 22}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


+---------+----------+-----------------------+
| Field   | Name     | Description           |
+=========+==========+=======================+
| [31:10] | WATCHDOG | watchdog down counter |
+---------+----------+-----------------------+

MMIO_INST_SG_COUNT_0
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x38 = 0x00000038`

    Stepgen 0 count

    .. wavedrom::
        :caption: MMIO_INST_SG_COUNT_0

        {
            "reg": [
                {"name": "sg_count_0[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_SG_COUNT_1
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x3c = 0x0000003c`

    Stepgen 1 count

    .. wavedrom::
        :caption: MMIO_INST_SG_COUNT_1

        {
            "reg": [
                {"name": "sg_count_1[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_SG_COUNT_2
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x40 = 0x00000040`

    Stepgen 2 count

    .. wavedrom::
        :caption: MMIO_INST_SG_COUNT_2

        {
            "reg": [
                {"name": "sg_count_2[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_SG_COUNT_3
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x44 = 0x00000044`

    Stepgen 3 count

    .. wavedrom::
        :caption: MMIO_INST_SG_COUNT_3

        {
            "reg": [
                {"name": "sg_count_3[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_SG_COUNT_4
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x48 = 0x00000048`

    Stepgen 4 count

    .. wavedrom::
        :caption: MMIO_INST_SG_COUNT_4

        {
            "reg": [
                {"name": "sg_count_4[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_SG_COUNT_5
^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x4c = 0x0000004c`

    Stepgen 5 count

    .. wavedrom::
        :caption: MMIO_INST_SG_COUNT_5

        {
            "reg": [
                {"name": "sg_count_5[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_WALLCLOCK
^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x50 = 0x00000050`

    wallclock time

    .. wavedrom::
        :caption: MMIO_INST_WALLCLOCK

        {
            "reg": [
                {"name": "wallclock[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_GPIOS_IN
^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x54 = 0x00000054`

    gpios in

    .. wavedrom::
        :caption: MMIO_INST_GPIOS_IN

        {
            "reg": [
                {"name": "gpios_in[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_ENC_COUNT_0
^^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x58 = 0x00000058`

    Encoder 0 count

    .. wavedrom::
        :caption: MMIO_INST_ENC_COUNT_0

        {
            "reg": [
                {"name": "enc_count_0[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_ENC_COUNT_1
^^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x5c = 0x0000005c`

    Encoder 1 count

    .. wavedrom::
        :caption: MMIO_INST_ENC_COUNT_1

        {
            "reg": [
                {"name": "enc_count_1[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_ENC_COUNT_2
^^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x60 = 0x00000060`

    Encoder 2 count

    .. wavedrom::
        :caption: MMIO_INST_ENC_COUNT_2

        {
            "reg": [
                {"name": "enc_count_2[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_ENC_COUNT_3
^^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x64 = 0x00000064`

    Encoder 3 count

    .. wavedrom::
        :caption: MMIO_INST_ENC_COUNT_3

        {
            "reg": [
                {"name": "enc_count_3[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_ENC_COUNT_4
^^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x68 = 0x00000068`

    Encoder 4 count

    .. wavedrom::
        :caption: MMIO_INST_ENC_COUNT_4

        {
            "reg": [
                {"name": "enc_count_4[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


MMIO_INST_ENC_COUNT_5
^^^^^^^^^^^^^^^^^^^^^

`Address: 0x00000000 + 0x6c = 0x0000006c`

    Encoder 5 count

    .. wavedrom::
        :caption: MMIO_INST_ENC_COUNT_5

        {
            "reg": [
                {"name": "enc_count_5[31:0]", "bits": 32}
            ], "config": {"hspace": 400, "bits": 32, "lanes": 1 }, "options": {"hspace": 400, "bits": 32, "lanes": 1}
        }


