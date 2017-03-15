
--[[
m = Map("cgminer", translate(""), "")

conf = m:section(TypedSection, "cgminer", "miner")
conf.anonymous = true
conf.addremove = false
--]]
--pooluser = conf:option(Value, "pooluser", translate("Bitmain User ID") ,translate(""))


f = SimpleForm("Status", translate("Device Status"))
f.reset = false
f.submit = false

stats = f:section(Table, luci.controller.cgminer.summary(), "Status")
stats:option(DummyValue, "elapsed", translate("Elapsed"))
stats:option(DummyValue, "ghs5s", translate("MH/S(5s)"))
stats:option(DummyValue, "ghsav", translate("MH/S(avg)"))
--stats:option(DummyValue, "temp_l", translate("Temp(PCB)"))
--stats:option(DummyValue, "temp_e", translate("Temp(Chip)"))

t0 = f:section(Table, luci.controller.cgminer.pools("r1"), translate("Pools"))
t0:option(DummyValue, "pool", translate("Pool"))
t0:option(DummyValue, "url", translate("URL"))
t0:option(DummyValue, "user", translate("User"))
t0:option(DummyValue, "status", translate("Status"))
t0:option(DummyValue, "diff", translate("Diff"))
t0:option(DummyValue, "getworks", translate("GetWorks"))
t0:option(DummyValue, "priority", translate("Priority"))
t0:option(DummyValue, "accepted", translate("Accepted"))
t0:option(DummyValue, "diff1shares", translate("Diff1#"))          
t0:option(DummyValue, "diffaccepted", translate("DiffA#"))              
t0:option(DummyValue, "diffrejected", translate("DiffR#"))              
t0:option(DummyValue, "diffstale", translate("DiffS#"))
t0:option(DummyValue, "rejected", translate("Rejected"))
t0:option(DummyValue, "discarded", translate("Discarded"))
t0:option(DummyValue, "stale", translate("Stale"))
t0:option(DummyValue, "lastsharedifficulty", translate("LSDiff"))
t0:option(DummyValue, "lastsharetime", translate("LSTime"))

local data = {}
local flag
flag,data = luci.controller.cgminer.blocks()

if(flag == 0 or data == {}) then
	bls = f:section(Table, data, "Found Blocks")
	bls:option(DummyValue, "Comment", "")
	
else
	bls = f:section(Table, data, "FoundBlocks")
	bls:option(DummyValue, "blockInfo", translate("Block Info"))
end

local apply = luci.http.formvalue("cbi.apply")
if apply then
    io.popen("/etc/init.d/cgminer restart")
end
    
return f
