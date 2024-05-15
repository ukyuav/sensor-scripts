--[[
PTH (Samamet) Serial Sensor Reading

Read the data from the serial line that the PTH is connected to, then decode the
messages from it, and log the data to the autopilots BIN file.

Author: Justin Tussey
Last Updated: 2024-05-15
]] --


-- global variable to keep track of how many iterations of the loop occur
-- without receiving data
local loops_without_data = 0

-- initialize serial connection
local BAUD_RATE = 9600

-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local PORT = assert(serial:find_serial(0),"Could not find Scripting Serial Port")

-- begin the serial port
PORT:begin(BAUD_RATE)
PORT:set_flow_control(0)

-- Calculate checksum by taking string from '$' to '*' (but not including them),
-- then XORing each of the ASCII characters in the string with the next one in
-- the string. Then compare the result in hex with the 2 character hex code
-- after the "*"
function verify_checksum(message_string)
  -- take the sub-string from (but not including) "$" and "*"
  local data_string = message_string:match("%$(.*)%*")

  -- extracts the the two characters after the '*' in the message string, and
  -- only accepts valid "2 character" hex numbers, ie: 4A. Then take that string,
  -- then convert it to an integer (16 specifies that our input string is a hex
  -- number)
  local incoming_checksum = tonumber(message_string:match("%*([0-9A-F][0-9A-F])"), 16)

  -- starting value of zeros, which will not effect the first XOR
  local checksum = 0x0

  -- Take the message string and convert it to bytes, then place into curly
  -- braces to convert it to an array of bytes, where each byte is an ASCII
  -- character. This allows us to iterate over it and do bitwise operations on
  -- it (lua does not support bitwise ops on strings/chars)
  local string_bytes = { data_string:byte(1, #data_string) }

  -- Iterate over the character array, and XOR each character with the
  -- previous "summed" XOR operations
  for i = 1, #string_bytes do
    checksum = (checksum ~ string_bytes[i])
  end

  if checksum ~= incoming_checksum then
    return false
  else
    return true
  end
end

-- Take the data from the verified message string and split up the string, using
-- a comma as a delimiter. Then take the measurements from the data and log them
-- to the BIN file.
function log_data(message_string)
  -- take the sub-string from (but not including) "$" and "*"
  local data_string = message_string:match("%$(.*)%*")
  local data_table = {}

  -- take the string, match up until the first comma, place that substring
  -- into the table, then repeat for the rest of the string.
  -- The regex is matching all characters up until it finds a ','. The "+" at
  -- the end allows matching more than once
  for str in string.gmatch(data_string, "([^" ..",".. "]+)") do
    table.insert(data_table, str)
  end

  local measurements_table={}
  for i=3,12,2 do
    table.insert(measurements_table, data_table[i])
  end

  logger:write('SAMA', 'pres,temp1,temp2,hum,temp3', -- section name and labels
               'NNNNN',                              -- data type (char[16])
               'POO%O',                              -- units,(Pa, C, C, % (humidity), C)
               '-----',                              -- multipliers (- signifies no multiplier)
               measurements_table[1],                -- data for labels
               measurements_table[2],
               measurements_table[3],
               measurements_table[4],
               measurements_table[5])

-- report data to Mission Planner, not necessary all the time
--   gcs:send_text(7, "\r\npres:" .. string.format(" %.2f \r\n", measurements_table[1]) ..
--                    "temp1:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "temp2:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "hum:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "temp3:" .. string.format(" %.2f \r\n", measurements_table[2])
--   )
-- end



function update()

  -- If there is one or more loops that are unsuccessful, send an error message
  -- to Mission Planner and write all zeros to the log file
  if loops_without_data > 0 then
    gcs:send_text(0, "ERROR: PTH has failed to send data for ".. string.format("%d", loops_without_data) .. " loops")

    -- write zeros to BIN file to make it clear that the sensor is disconnected
    logger:write('SAMA', 'pres,temp1,temp2,hum,temp3', -- section name and labels
               'NNNNN',                              -- data type (char[16])
               'POO%O',                              -- units,(Pa, C, C, % (humidity), C)
               '-----',                              -- multipliers (- signifies no multiplier)
               '0',                                  -- data for labels
               '0',
               '0',
               '0',
               '0')
  end

  -- increment the number of unsuccessful loops. If the loop is unsuccessful
  -- (n_bytes <= 0 so it does not enter the while loop), it will not be set to
  -- zero and trigger the above if statement on the next loop
  loops_without_data = loops_without_data + 1

  local n_bytes = PORT:available()
  while n_bytes > 0 do
    -- only read a max of 515 bytes in a go
    -- this limits memory consumption
    local buffer = {} -- table to buffer data
    local bytes_target = n_bytes - math.min(n_bytes, 512)
    while n_bytes > bytes_target do
      table.insert(buffer,PORT:read())
      n_bytes = n_bytes - 1
    end

    local data = string.char(table.unpack(buffer))
    if (verify_checksum(data)) then
      log_data(data)
    else
      gcs:send_text(0, "ERROR: PTH Data failed checksum, check sensor!")
    end

    -- set unsuccessful loop count to 0, since we completed a loop
    loops_without_data = 0
  end


  return update, 1000 -- reschedules the loop every 1000ms (temporary)
end

return update() -- run immediately before starting to reschedule
