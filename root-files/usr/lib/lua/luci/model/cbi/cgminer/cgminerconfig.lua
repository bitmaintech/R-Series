m = Map("cgminer-u3", translate("Configuration"), translate(""))

conf = m:section(TypedSection, "cgminer", "")
conf.anonymous = true
conf.addremove = false

conf:tab("default", translate("General Settings"))

pool1url = conf:taboption("default", Value, "pool1url", translate("Pool 1"))
pool1user = conf:taboption("default", Value,"pool1user", translate("Pool1 worker"))
pool1pw = conf:taboption("default", Value,"pool1pw", translate("Pool1 password"))
conf:taboption("default", DummyValue, "", translate(""))
pool2url = conf:taboption("default", Value, "pool2url", translate("Pool 2"))
pool2user = conf:taboption("default", Value, "pool2user", translate("Pool2 worker"))
pool2pw = conf:taboption("default", Value, "pool2pw", translate("Pool2 password"))
conf:taboption("default", DummyValue, "", translate(""))
pool3url = conf:taboption("default", Value, "pool3url", translate("Pool 3"))
pool3user = conf:taboption("default", Value, "pool3user", translate("Pool3 worker"))
pool3pw = conf:taboption("default", Value, "pool3pw", translate("Pool3 password"))
conf:taboption("default", DummyValue, "", translate(""))

pb = conf:taboption("default", ListValue, "pool_balance", translate("Pool Balance(Default: Failover)"))
pb.default = "  "
pb:value("  ", translate("Failover"))
pb:value("--balance", translate("Balance"))
pb:value("--load-balance", translate("Load Balance"))

--[[pb = conf:option(ListValue, "bitmain_nobeeper", translate("Beeper ringing(Default: true)"))
pb.default = "  "
pb:value("  ", translate("true"))
pb:value("--bitmain-nobeeper", translate("false"))

pb = conf:option(ListValue, "bitmain_notempoverctrl", translate("Stop running when temprerature is over 80 degrees centigrade(Default: true)"))
pb.default = "  "
pb:value("  ", translate("true"))
pb:value("--bitmain-notempoverctrl", translate("false"))
--]]
--cf = conf:option(Value, "chip_frequency", translate("Chip Frequency(Default: 300)"))

--mc = conf:option(Value, "miner_count", translate("Miner Count(Default: 24)"))
--api_allow = conf:option(Value, "api_allow", translate("API Allow(Default: W:127.0.0.1)"))

--target=conf:option(Value, "target", translate("Target Temperature"))
--overheat=conf:option(Value, "overheat", translate("Overheat Cut Off Temperature"))

--more_options = conf:option(Value, "more_options", translate("More Options(Default: --quiet)"))


conf:tab("advanced", translate("Advanced Settings"))
pb = conf:taboption("advanced", ListValue, "freq", translate("Frequency")) 
pb.default = "0.66:206.25:1006"
pb:value("0.55:243.75:1306", translate("243.75M"))
pb:value("0.57:237.5:1286", translate("237.5M"))
pb:value("0.61:225:0882", translate("225M"))
pb:value("0.62:218.75:1106", translate("218.75M")) 
pb:value("0.64:212.5:1086", translate("212.5M"))
pb:value("0.66:206.25:1006", translate("206.25M (default)"))
pb:value("0.68:200:0782", translate("200M"))
pb:value("0.69:196.88:1f07", translate("196.88M"))
pb:value("0.71:193.75:0f03", translate("193.75M"))
pb:value("0.78:175:0d83", translate("175M"))
pb:value("0.91:150:0b83", translate("150M"))
pb:value("1.09:125:0983", translate("125M"))
pb:value("1.36:100:0783", translate("100M"))
  
pb = conf:taboption("advanced", Value, "voltage", translate("voltage"),"Modify voltage and Save &#38; Apply, then need to Power off and Restart")
conf:option(Value,"voltage", "voltage")

local apply = luci.http.formvalue("cbi.apply")
if apply then
   io.popen("/etc/init.d/cgminer-u3 stop")
   -- os.execute("sleep" ..5000)
   -- socket.select(nil, nil, 3)
   local t0 = os.clock()
   while os.clock() - t0 <= 3  do
   
   end
   
   io.popen("/etc/init.d/cgminer-u3 start")
end

return m
