--[[

I2C Sensor reading (Small file size)

This program communicates with all on board airspeed sensors and reports values in the .bin file

Author: Ryan Prince
Last Updated: 29 Feb 2024

]]--
local log_data = {}
local i2c_bus = i2c:get_device(0, 0x00)
i2c_bus:set_retries(10)
gcs:send_text(7, "i2c_readSensors Script Started!")

sensor_addr = {0x28, 0x38, 0x48, 0x58, 0x68, 0x78}

function update()
  for key, value in pairs(sensor_addr) do
  i2c_bus:set_address(sensor_addr[key])

    if i2c_bus:read_registers(0) then
      returnTable = i2c_bus:read_registers(0, 2)
      msg =  (returnTable[1] << 8 | returnTable[2]) & 0x3FFF
      log_data[key] = tostring((4.0*msg)/0x3FFF - 2)
    end
  end

  logger:write('SENS','s1,s2,s3,s4,s5,s6','NNNNNN', log_data[1], log_data[2], log_data[3], log_data[4], log_data[5], log_data[6])

  i2c_bus:set_address(0x00)
  return update, 200
end

return update()