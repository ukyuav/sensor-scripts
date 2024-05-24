--[[
Anemometer Serial Sensor Reading

Read the data from the serial line that the anemometer, decode and extract the
data we need, and log it to the autopilot.

Uses CV7-OEM Ultrasonic Wind Sensor from LCJ Capteurs

Author: Justin Tussey
Last Updated: 2024-05-20
]]--

-- Sends Data every 512 milliseconds, Baud rate of 4800

-- variable to count iterations without getting message
local loops_since_data_received = 0

-- error type table
local ERROR_LIST = {
  "No data received",      -- 1
  "Checksum fail",         -- 2
  "Data parsing fail",     -- 3
}

-- initialize serial connection
local BAUD_RATE = 4800

-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local PORT = assert(serial:find_serial(0),"Could not find Scripting Serial Port")

-- begin the serial port
PORT:begin(BAUD_RATE)
PORT:set_flow_control(0)


local message_table = {}


function verify_checksum(message_string)
  -- take the sub-string from (but not including) "$" and "*"
  local data_string = message_string:match("%$(.*)%*")

  -- check that the regex successfully parsed the string
  if data_string == nil then
    return false
  end

  -- extracts the the two characters after the '*' in the message string, and
  -- only accepts valid "2 character" hex numbers, ie: 4A. Then take that string
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

function parse_data(message_string)
  -- take the sub-string from (but not including) "$" and "*"
  local data_string = message_string:match("%$(.*)%*")

  -- check that the regex successfully parsed the string
  -- (unlikely since we already checked but it doesn't hurt)
  if data_string == nil then
    return false
  end

  local message_type = message_string:match("%$(.-),")

  local data_table = {}

  -- take the string, match up until the first comma, place that substring
  -- into the table, then repeat for the rest of the string.
  -- The regex is matching all characters up until it finds a ",". The "+" at
  -- the end allows matching more than once
  for str in string.gmatch(data_string, "([^" ..",".. "]+)") do
    table.insert(data_table, str)
  end

  if message_type == "IIMWV" then
    return log_wind_speed(data_table)
  elseif message_type == "WIXDR" then
    return log_wind_temp(data_table)
  end

  return true
end

function log_wind_speed(message_table)

  if #message_table ~= 6 then
    return false
  end

  logger:write('ANEM', 'angle,speed,error',  -- section name and labels
               'NNN',                        -- data type (char[16])
               message_table[2],        -- data for labels
               message_table[4],
               "Normal")


  return true
end

function log_wind_temp(message_table)
  if #message_table ~= 4 then
    return false
  end

  return true
end

-- called when an error is detected, and writes all zeros to the BIN file
function log_error(error_type)
  logger:write('ANEM', 'angle,speed,error',  -- section name and labels
               'NNN',                        -- data type (char[16])
               "0",                          -- data for labels
               "0",
               error_type)
end


function update()
  local n_bytes = PORT:available()

  if n_bytes <= 0 then
    loops_since_data_received = loops_since_data_received + 1
    if loops_since_data_received > 11 then
      log_error(ERROR_LIST[1])
      gcs:send_text(0, "ERROR: Disconnected Sensor")
      -- clear incomplete message (if there is one)
      message_table = {}
    end
    return update, 100
  end

  loops_since_data_received = 0
  while n_bytes > 0 do
    local byte = PORT:read()
    if byte == 0x0A then
      table.insert(message_table, byte)
      local message_string = string.char(table.unpack(message_table))
      if not (verify_checksum(message_string)) then
        log_error(ERROR_LIST[2])
        gcs:send_text(0, "ERROR: PTH Data failed checksum")
        message_table = {}
        return update, 100
      end
      if not (parse_data(message_string)) then
        log_error(ERROR_LIST[3])
        gcs:send_text(0, "ERROR: Failed to parse data")
        message_table = {}
        return update, 100
      end
      gcs:send_text(7, message_string)
      -- reset for the next message
      message_table = {}
      return update, 100
    end
    table.insert(message_table, byte)
    n_bytes = n_bytes - 1
  end
  return update, 100 -- schedule the update function to
end

return update()
