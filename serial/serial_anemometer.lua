--[[
Anemometer Serial Sensor Reading

Read the data from the serial line that the anemometer, decode and extract the
data we need, and log it to the autopilot.

Uses CV7-OEM Ultrasonic Wind Sensor from LCJ Capteurs

Sends Data every 512 milliseconds, Baud rate of 4800

Author: Justin Tussey
Last Updated: 2024-06-18
]]--

-- Global Constants --

-- initialize serial connection
local BAUD_RATE = 4800

-- Max length of the messages from the anemometer
local MAX_MESSAGE_LENGTH = 31

-- Return rate and calculations
local SCHEDULE_RATE = 100 --milliseconds
local TIME_BETWEEN_DATA = 512 --milliseconds
-- number of how many loops we need for us to properly flag that the sensor is
-- not sending data (// is floor division (removes decimal))
local LOOPS_TO_FAIL = (TIME_BETWEEN_DATA // SCHEDULE_RATE) + (1)

-- error type table
-- must be 16 characters or less
local ERROR_LIST = {
  "No data received", -- 1
  "Checksum fail",    -- 2
  "Parsing fail",     -- 3
  "Invalid frame",    -- 4
  -- "Alarm received"    -- 5
}

-- info about the messages we receive
local MESSAGE_INFO = {
  ["IIMWV"] = {
    length = 28,
    fields = 6,
    measurements = 2
  },
  -- ["WIXDR"] = {
  --   length = 22,
  --   fields = 4,
  --   measurements = 1
  -- }
}


-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local PORT = assert(serial:find_serial(0),"Could not find Scripting Serial Port")
-- local PORT = assert(serial:find_serial(1),"Could not find Scripting Serial Port")

-- begin the serial port
PORT:begin(BAUD_RATE)
PORT:set_flow_control(0)

-- Global Variables --
-- variable to count iterations without getting message
local loops_since_data_received = 0


-- Take in string and verify that it is a valid message frame. Specifically a
-- message in the NMEA-0183 format, which starts with "$" and ends with <CR><LF>
-- (DOS line ending)
---@param message_string string
---@return boolean
function verify_valid_frame(message_string)
  -- get the first character in the message, which should be "$" if valid
  local head = string.sub(message_string, 1, 1)

  -- get the last two characters in the message, which should be <CR><LF>
  -- ("\r\n") if valid
  local tail = string.sub(message_string, #message_string-1, #message_string)

  if (head ~= "$") then
    return false
  elseif (tail ~= "\r\n") then
    return false
  end

  return true
end

-- Checks if a message is "useful". Make sure its one of the messages we would
-- like to parse and log
---@param  message_string string
---@return boolean
function is_message_useful(message_string)
  local message_type = message_string:match("%$(.-),")

  if message_type == nil then
    return false
  elseif message_type == "IIMWV" then
    return true
  end
  return false
end

-- Calculate checksum by taking sub-string from $ to * (but not including
-- them), then XORing each of the ASCII characters in the string with the next
-- one in the string. Then compare the result in hex with the 2 character hex
-- code after the *. Returns a boolean value of whether it passed the check.
---@param  message_string string
---@return boolean
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

  -- Take the string of the checksum, convert it to a number, specifying that its
  -- a hexadecimal number
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

  -- check if checksum matches, return true or false
  return (checksum == incoming_checksum)
end

-- Take the data from a verified message string and split up the string, using
-- a comma as a delimiter. Then take the measurements from the data and log them
-- to the BIN file. Return whether the parsing failed or passed
---@param  message_string string
---@return boolean
function parse_data(message_string)
  -- take the sub-string from (but not including) "$" and "*"
  local data_string = message_string:match("%$(.*)%*")

  -- check that the regex successfully parsed the string
  -- (unlikely since we already checked but it doesn't hurt)
  if data_string == nil then
    return false
  end

  -- extract the message type header from the message
  local message_type = message_string:match("%$(.-),")

  -- take the string, match up until the first comma, place that substring
  -- into the data_table, then repeat for the rest of the string.
  -- The regex is matching all characters up until it finds a ",". The "+" at
  -- the end allows matching more than once
  local data_table = {}
  for str in string.gmatch(data_string, "([^" ..",".. "]+)") do
    table.insert(data_table, str)
  end

  -- check if we extracted the expected number of data fields for this message
  -- type
  if #data_table ~= MESSAGE_INFO[message_type].fields then
    return false
  end

  -- extract the measurements (numbers) from the fields in data_table
  local measurement_table={}
  for i = 1, #data_table do
    local m = string.match(data_table[i], "%d*%.%d*")
    if m ~= nil then
      table.insert(measurement_table, m)
    end
  end

  -- check if we extracted the expected number of measurements
  if #measurement_table ~= MESSAGE_INFO[message_type].measurements then
    return false
  end

  if message_type == "IIMWV" then
    return log_wind_speed(measurement_table)
  end

   return false
end

-- take the input table, then write that data to the BIN file, with the
-- appropriate names and labels
---@param measurement_table table
---@return boolean
function log_wind_speed(measurement_table)
  -- check if we have the expected number of measurements
  if #measurement_table ~= MESSAGE_INFO["IIMWV"].measurements then
    return false
  end

  -- gcs:send_text(7, "angle: " .. tostring(measurement_table[1]) .. " speed: " .. tostring(measurement_table[2]))

  logger:write('ANEM', 'angle,speed,error',  -- section name and labels
               'NNN',                        -- data type (char[16])
               measurement_table[1],             -- data for labels
               measurement_table[2],
               'Normal')
  return true
end

-- called when an error is detected, and writes all zeros to the BIN file and
-- writes what kind of error was experienced. error_type parameter must be 16
-- bytes long or less
---@param  error_type string
function log_error(error_type)
  logger:write('ANEM', 'angle,speed,error',  -- section name and labels
               'NNN',                        -- data type (char[16])
               '0',                          -- data for labels
               '0',
               error_type)
end

-- primary loop. Gets message from serial interface, verifies that it is valid,
-- then logs the data to the autopilots BIN file
---@return function
---@return integer
function update()
  local n_bytes = PORT:available()

  -- Check if there are any bytes available, if not log an error after the
  -- specified number of failed loops
  if n_bytes <= 0 then
    loops_since_data_received = loops_since_data_received + 1
    if loops_since_data_received >= LOOPS_TO_FAIL then
      log_error(ERROR_LIST[1])
      gcs:send_text(0, "ERROR ANEM: Disconnected Sensor")
    end
    return update, SCHEDULE_RATE
  end

  -- get MAX_MESSAGE_LENGTH bytes from the serial line, and place them into a
  -- string
  local message_string = PORT:readstring(MAX_MESSAGE_LENGTH)

  -- make sure we got a message from the serial line
  if (message_string == nil or #message_string <= 0) then
    return update, SCHEDULE_RATE
  end

  -- clear loops since data received since we passed the checks to verify we got
  -- a message
  loops_since_data_received = 0

  -- check if we have a valid data frame, which checks for the NMEA-0183
  -- sentence start and ending characters. If not valid (which means we do not
  -- have a complete message) clear the queue to realign messages and return
  if not (verify_valid_frame(message_string)) then
    -- Read the available bytes in the queue and do nothing with them.
    -- We effectively clear the queue
    PORT:readstring(PORT:available():toint())
    log_error(ERROR_LIST[4])
    gcs:send_text(0, "ERROR ANEM: Invalid message frame")
    gcs:send_text(7, message_string)
    return update, SCHEDULE_RATE
  end

  -- check if message is one we want to handle, skip if not
  if not (is_message_useful(message_string)) then
    return update, SCHEDULE_RATE
  end

  -- verify the checksum in the message to check if the data has been corrupted
  -- log an error if we fail the check
  if not (verify_checksum(message_string)) then
    log_error(ERROR_LIST[2])
    gcs:send_text(0, "ERROR ANEM: Data failed checksum")
    gcs:send_text(7, message_string)
    return update, SCHEDULE_RATE
  end

  -- parse and log the data, log error if we experience an issue. Most likely
  -- means that the data was corrupted and was not caught by the checksum
  -- verification (unlikely)
  if not (parse_data(message_string)) then
    log_error(ERROR_LIST[3])
    gcs:send_text(0, "ERROR ANEM: Failed to parse data")
    gcs:send_text(7, message_string)
    return update, SCHEDULE_RATE
  end

  -- gcs:send_text(7, message_string)
  -- reschedule the update function to run every SCHEDULE_RATE milliseconds
  return update, SCHEDULE_RATE
end

return update()
