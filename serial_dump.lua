-- this script reads data from a serial port and sends it to the Misson Planner
-- output. (Modified from the Serial_Dump.lua example provided by Ardupilot)
-- "Author": Justin Tussey

local baud_rate = 9600

-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local port = assert(serial:find_serial(0),"Could not find Scripting Serial Port")

-- begin the serial port
port:begin(baud_rate)
port:set_flow_control(0)

function update() -- this is the loop which periodically runs

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

    -- write as plain text
    gcs:send_text(7, string.char(table.unpack(buffer)))

  end

  return update, 100 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
