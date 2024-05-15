--[[
PTH (Samamet) Serial Sensor Reading

Read the data from the serial line that the PTH is connected to, then decode the
messages from it, and log the data to the autopilots BIN file.

Author: Justin Tussey
Last Updated: 2024-05-15
]] --


-- initialize serial connection

local baud_rate = 9600

-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local port = assert(serial:find_serial(0),"Could not find Scripting Serial Port")

-- begin the serial port
port:begin(baud_rate)
port:set_flow_control(0)

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
  -- braces to convert it to an array of bytes, where each byte is an ascii
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

end


function update()

  local n_bytes = port:available()
  while n_bytes > 0 do
    -- only read a max of 515 bytes in a go
    -- this limits memory consumption
    local buffer = {} -- table to buffer data
    local bytes_target = n_bytes - math.min(n_bytes, 512)
    while n_bytes > bytes_target do
      table.insert(buffer,port:read())
      n_bytes = n_bytes - 1
    end

    if (verify_checksum(string.char(table.unpack(buffer)))) then
      -- continue
    else
      gcs:send_text(0, "ERROR: PTH Data failed checksum, check sensor!")
    end

  end


  return update, 1000 -- reschedules the loop every 1000ms (temporary)
end

return update() -- run immediately before starting to reschedule
