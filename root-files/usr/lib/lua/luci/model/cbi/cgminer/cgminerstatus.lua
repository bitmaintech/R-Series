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

t1 = f:section(Table, luci.controller.cgminer.summary_u3(), translate("Summary"))
t1:option(DummyValue, "num", translate("Num"))
t1:option(DummyValue, "ghs5s", translate("GH/S(5s)"))
t1:option(DummyValue, "ghsav", translate("GH/S(avg)"))

t0 = f:section(Table, luci.controller.cgminer.pools("u3"), translate("Pools"))
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

t2 = f:section(Table, luci.controller.cgminer.stats_u3(), translate("Devices"))
t2:option(DummyValue, "stats", translate("No"))
t2:option(DummyValue, "ghs5s", translate("GH/S(5s)"))
t2:option(DummyValue, "ghsav", translate("GH/S(avg)"))

--[[
t1 = f:section(Table, luci.controller.cgminer.devs(), translate("AntMiner"))
t1:option(DummyValue, "chain", translate("Chain#"))
t1:option(DummyValue, "asic", translate("ASIC#"))
t1:option(DummyValue, "frequency", translate("Frequency"))
t1:option(DummyValue, "fan", translate("Fan"))
t1:option(DummyValue, "temp", translate("Temp"))
t1:option(DummyValue, "status", translate("ASIC status"))
t1 = f:section(Table, luci.controller.cgminer.devs(), translate(""))
t1:option(DummyValue, "fan", translate("Fan#"))



t1 = f:section(Table, luci.controller.cgminer.devs(), translate("AntMiner"))  
t1:option(DummyValue, "chain", translate("Chain#") )
t1:option(DummyValue, "asic", translate("ASIC#"))
t1:option(DummyValue, "frequency", translate("Frequency"))
t1:option(DummyValue, "temp", translate("Temp"))
t1:option(DummyValue, "status", translate("ASIC status"))

t1 = f:section(Table, luci.controller.cgminer.devs(), translate(""))  
t1:option(DummyValue, "fan_name", translate("Fan#") )
t1:option(DummyValue, "fan1", translate("Fan1") )
t1:option(DummyValue, "fan2", translate("Fan2") )
t1:option(DummyValue, "fan3", translate("Fan3") )
t1:option(DummyValue, "fan4", translate("Fan4") )
]]--

return f