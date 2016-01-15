m = Map("cgminer", translate(""), "<a href='http://www.antpool.com' target='_blank'>My SOLO History</a>")

conf = m:section(TypedSection, "cgminer", "miner")
conf.anonymous = true
conf.addremove = false

pooluser = conf:option(Value, "pooluser", translate("Bitmain User ID") ,translate("MyAntRouter@bitmain.com is a special user ID which is combined with the Btimain's user ID of original buyer who bought this router from Bitmain directly \
Please set above Bitmain user ID as your own Btimain user ID.\
If your router found a block whose difficult meets the bitcoin network, antpool will send a Email to the above Email address to you, and then you can get around the total block earning.\
additional:When the router is running you can get Bitmain points.(1 point/GHz/H), which can be used to buy something on www.bitmiantech.com or www.hashnest.com."))


stats = m:section(Table, luci.controller.cgminer.summary(), "Miner Status")
stats:option(DummyValue, "elapsed", translate("Elapsed"))
stats:option(DummyValue, "ghs5s", translate("GH/S(5s)"))
stats:option(DummyValue, "ghsav", translate("GH/S(avg)"))

local data = {}
local flag
flag,data = luci.controller.cgminer.blocks()

if(flag == 0 or data == {}) then
	bls = m:section(Table, data, "Found Blocks")
	bls:option(DummyValue, "Comment", "")
	
else
	bls = m:section(Table, data, "FoundBlocks")
	bls:option(DummyValue, "blockInfo", translate("Block Info"))
end

local apply = luci.http.formvalue("cbi.apply")
if apply then
    io.popen("/etc/init.d/cgminer restart")
end
    
return m
