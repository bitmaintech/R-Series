--[[
LuCI - Lua Configuration Interface

Copyright 2013 Xiangfu
Copyright 2008 Steven Barth <steven@midlink.org>
Copyright 2008 Jo-Philipp Wich <xm@leipzig.freifunk.net>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

$Id$
]]--
f = SimpleForm("cgminerstatus", translate("Device Status"))
f.reset = false
f.submit = false

stats = f:section(Table, luci.controller.cgminer.stats_r2(), "Miner Status")
stats:option(DummyValue, "elapsed", translate("Elapsed"))
stats:option(DummyValue, "ghs5s", translate("GH/S(5s)"))
stats:option(DummyValue, "ghsav", translate("GH/S(avg)"))
stats:option(DummyValue, "temp_l", translate("Temp(PCB)"))
stats:option(DummyValue, "temp_e", translate("Temp(Chip)"))


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




return f