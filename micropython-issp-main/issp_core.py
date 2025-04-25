import machine
import micropython

@micropython.viper
def _drive_clock(cycles: int):
    while cycles > 0:
        sclk(1)
        sclk(0)
        cycles -= 1


@micropython.viper
def _on_callback(t):
    sclk(1)
    t.callback(_off_callback)


@micropython.viper
def _off_callback(t):
    sclk(0)
    t.deinit()


@micropython.viper
def wait_and_poll():
    """Perform a wait-and-poll.

    Uses a timer to generate the first clock cycle.
    """
    sdata.init(machine.Pin.IN)

    timer.init(freq=50000, callback=_on_callback)

    ret = int(machine.time_pulse_us(sdata, 1, _WAIT_AND_POLL_TIMEOUT_US))
    if ret == -2:
        raise RuntimeError('Timed out waiting for SDATA to go high')
    elif ret == -1:
        raise RuntimeError('Timed out waiting for SDATA to go low')

    sdata(0)
    sdata.init(machine.Pin.OUT)
    _drive_clock(40)


@micropython.viper
def write_noop():
    """Send 22 zeros to the target."""
    sdata(0)
    _drive_clock(22)


@micropython.viper
def reset_send_magic():
    """Reset the target using XRES pin, and send the sequence to put target into ISSP mode.

    The first 22 bits of the Initialize-1 vector is sent.

    Note: this method initialize pins, and also invokes the garbage collector, before sending the magic bits.

    Timing note: The reset procedure has tight timing requirements. This method uses busy loops to generate delays.
    Verify timing with a logic analyzer and tweak as required.
    """
    init_pins()
    gc.collect()

    sdata(0)
    sclk(0)
    xres(0)
    for _ in range(100):
        pass

    xres(1)
    for _ in range(100):
        pass

    xres(0)
    for _ in range(20):
        pass

    sdata(1)
    sclk(1)
    sclk(0)

    sclk(1)
    sclk(0)

    sdata(0)
    sclk(1)
    sclk(0)

    sclk(1)
    sclk(0)

    sdata(1)
    sclk(1)
    sclk(0)

    sdata(0)
    sclk(1)
    sclk(0)

    sdata(1)
    sclk(1)
    sclk(0)

    sdata(0)
    i = 0
    while i < 15:
        sclk(1)
        sclk(0)
        i += 1


@micropython.viper
def power_cycle_send_magic():
    """Cycle the target's power and send the sequence to put target into ISSP mode.

    The first 22 bits of the Initialize-1 vector is sent.

    Note: this method initialize pins, and also invokes the garbage collector, before sending the magic bits.

    Timing note: the power cycle procedure is somewhat less strict than the XRES reset procedure.
    However, there's the risk of not catching the high-to-low transition on sdata. Try tweaking the delay after
    power_enable.on().
    """
    power_off()
    gc.collect()
    time.sleep_ms(100)

    xres.init(machine.Pin.OUT, None)
    xres(0)
    power_enable.on()
    time.sleep_us(100)
    ret = int(machine.time_pulse_us(sdata, 1, _WAIT_AND_POLL_TIMEOUT_US))
    if ret == -2:
        raise RuntimeError('Timed out waiting for SDATA to go high')
    elif ret == -1:
        raise RuntimeError('Timed out waiting for SDATA to go low')

    sdata.init(machine.Pin.OUT, None)
    sclk.init(machine.Pin.OUT, None)

    sdata(1)
    sclk(1)
    sclk(0)

    sclk(1)
    sclk(0)

    sdata(0)
    sclk(1)
    sclk(0)

    sclk(1)
    sclk(0)

    sdata(1)
    sclk(1)
    sclk(0)

    sdata(0)
    sclk(1)
    sclk(0)

    sdata(1)
    sclk(1)
    sclk(0)

    sdata(0)
    i = 0
    while i < 15:
        sclk(1)
        sclk(0)
        i += 1


@micropython.viper
def _read_op(issp_op: int, address: int) -> int:
    i = 2
    while i >= 0:
        sdata((issp_op >> i) & 1)
        sclk(1)
        sclk(0)
        i -= 1

    i = 7
    while i >= 0:
        sdata((address >> i) & 1)
        sclk(1)
        sclk(0)
        i -= 1

    read_value = 0

    sdata.init(machine.Pin.IN)
    sclk(1)
    sclk(0)
    sclk(1)
    sclk(0)

    i = 7
    while i >= 0:
        read_value |= int(sdata()) << i
        sclk(1)
        sclk(0)
        i -= 1

    sclk(1)
    sclk(0)
    sdata.init(machine.Pin.OUT)
    return read_value


@micropython.viper
def _write_op(issp_op: int, address: int, value: int):
    i = 2
    while i >= 0:
        sdata((issp_op >> i) & 1)
        sclk(1)
        sclk(0)
        i -= 1

    i = 7
    while i >= 0:
        sdata((address >> i) & 1)
        sclk(1)
        sclk(0)
        i -= 1

    i = 7
    while i >= 0:
        sdata((value >> i) & 1)
        sclk(1)
        sclk(0)
        i -= 1

    sdata(1)
    i = 0
    while i < 3:
        sclk(1)
        sclk(0)
        i += 1


@micropython.viper
def read_memory(address: int) -> int:
    """Read a byte of memory at `address`."""
    return int(_read_op(_ISSP_OP_READ_MEM, address))


@micropython.viper
def read_register(address: int) -> int:
    """Read a byte of the register at `address`."""
    return int(_read_op(_ISSP_OP_READ_REG, address))


@micropython.viper
def write_memory(address: int, value: int):
    """Write a byte `value` to memory at `address`."""
    _write_op(_ISSP_OP_WRITE_MEM, address, value)


@micropython.viper
def write_register(address: int, value: int):
    """Write a byte `value` to register at `address`."""
    _write_op(_ISSP_OP_WRITE_REG, address, value)


@micropython.viper
def _write_vector(ops):
    ptr_ops = ptr8(ops)
    i = 0
    len_ops = int(len(ops))
    while i < len_ops:
        if ptr_ops[i] == 0:
            write_noop()
            i += 1
            continue
        assert i + 3 <= len_ops
        _write_op(ptr_ops[i], ptr_ops[i+1], ptr_ops[i+2])
        i += 3