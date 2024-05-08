--[[

I2C TCA9548 interface

This program uses the TCA Multiplexer to communicate to multiple devices with the same addresses

***Example included assumes 0x28 is connected channel 6 on the TCA module

Author: Ryan Prince
Last Updated: 07 Mar 2024

]]--
-- init i2c bus
local i2c_bus = i2c:get_device(0,0)
i2c_bus:set_retries(10)
gcs:send_text(7, "i2c_tca Script Started!")

-- for each TCA9548A, add an entry with its address
-- 0x70 is default, to add more set or reset A0, A1, A2
TCA_ADDRESSES = {
  0x70
}

-- opens the channel to the designated TCA module
-- ***Does not account for out of range inputs
function tcaselect(tca, channel)
  -- choose multiplexer from array
  i2c_bus:set_address(TCA_ADDRESSES[tca])
  -- set/open the correct channel
  i2c_bus:write_register(0x70, 1 << channel)
end

-- MAIN FUNCTION
function update()
  -- Example:
  -- select TCA module 1, channel 6
  tcaselect(1,6)
  -- once open use the address of the sensor
  i2c_bus:set_address(0x28)
  -- read_registers(begin at register, number of bytes to read)
  returnTable = i2c_bus:read_registers(0, 2)

  -- output data to MP Messages
  -- format data to remove first 2 bits
  msg =  (returnTable[1] << 8 | returnTable[2]) & 0x3FFF
  -- send_text(priority level (7 is Debug), text as a string formatted to hex)
  gcs:send_text(7, "Data on " .. "0x28: " .. string.format("%x", msg))
  -- end Example

  return update, 1000 -- reschedules the loop every 1000ms
end

return update() -- run immediately before starting to reschedule