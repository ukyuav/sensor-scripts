--[[
PTH (Samamet) Serial Sensor Reading

Read the data from the serial line that the PTH is connected to, then decode the
messages from it, and log the data to the autopilots BIN file.

Author: Justin Tussey
Last Updated: 2024-05-16
]] --

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

  -- check that the regex successfully parsed the string
  if data_string == nil then
    return false
  end

  -- extracts the the two characters after the '*' in the message string, and
  -- only accepts valid "2 character" hex numbers, ie: 4A. Then take that string,
  -- then convert it to an integer (16 specifies that our input string is a hex
  -- number)
  local incoming_checksum = message_string:match("%*([0-9A-F][0-9A-F])")

  -- check that the regex successfully parsed the string
  if incoming_checksum == nil then
    return false
  end

  incoming_checksum = tonumber(incoming_checksum, 16)

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
function parse_data(message_string)
  -- take the sub-string from (but not including) "$" and "*"
  local data_string = message_string:match("%$(.*)%*")

  -- check that the regex successfully parsed the string
  -- (unlikely since we already checked but it doesn't hurt)
  if data_string == nil then
    return false
  end


  local data_table = {}

  -- take the string, match up until the first comma, place that substring
  -- into the table, then repeat for the rest of the string.
  -- The regex is matching all characters up until it finds a ','. The "+" at
  -- the end allows matching more than once
  for str in string.gmatch(data_string, "([^" ..",".. "]+)") do
    table.insert(data_table, str)
  end

  -- make sure regex successfully splits by commas.
  -- (hopefully will not reach here since the checksum should fail before then)
  if #data_table ~= 12 then
    return false
  end

  local measurements_table={}
  for i=3,12,2 do
    table.insert(measurements_table, data_table[i])
  end

  -- return whether data input data matched needed format (table with 5
  -- elements)
  return log_data(measurements_table)

-- report data to Mission Planner, not necessary all the time
--   gcs:send_text(7, "\r\npres:" .. string.format(" %.2f \r\n", measurements_table[1]) ..
--                    "temp1:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "temp2:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "hum:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "temp3:" .. string.format(" %.2f \r\n", measurements_table[2])
--   )

end

-- take the input table, and check if it has 5 elements in it, then write that
-- data to the BIN file, with the appropriate names and units
function log_data(measurements_table)
  if #measurements_table ~= 5 then
    return false
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
  return true
end

-- called when an error is detected, and writes all zeros to the BIN file
function log_error()
    logger:write('SAMA', 'pres,temp1,temp2,hum,temp3', -- section name and labels
                 'NNNNN',                              -- data type (char[16])
                 'POO%O',                              -- units,(Pa, C, C, % (humidity), C)
                 '-----',                              -- multipliers (- signifies no multiplier)
                 '0', '0', '0', '0', '0') -- zeros for labels since there is an error with the PTH or its data
end


function update()
  local n_bytes = PORT:available()

  -- If there is one or more loops that are unsuccessful, send an error message
  -- to Mission Planner and write all zeros to the log file
  if n_bytes <= 0 then
    log_error()
    gcs:send_text(0, "ERROR: PTH has failed to send data")
    -- write zeros to BIN file to make it clear that the sensor is disconnected
  end

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
    -- check if checksum is valid
    if (verify_checksum(data)) then
      -- make sure that data is logged correctly
      if  not (parse_data(data)) then
        log_error()
        gcs:send_text(0, "ERROR: PTH data was not successfully parsed or not written to BIN file correctly!")
        gcs:send_text(0, "Incoming string: " .. data)
      end
    else
      log_error()
      gcs:send_text(0, "ERROR: PTH Data failed checksum, check sensor!")
      gcs:send_text(0, "Incoming string: " .. data)
    end
  end

  return update, 1000 -- reschedules the loop every 1000ms (1 second, max since sensor only sends 1 message every second)
end

return update() -- run immediately before starting to reschedule