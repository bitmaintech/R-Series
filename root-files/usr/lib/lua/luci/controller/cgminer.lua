--[[
LuCI - Lua Configuration Interface

Copyright 2013 Xiangfu

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

$Id$
]]--

module("luci.controller.cgminer", package.seeall)

function index()
   entry({"admin", "miner", "miner"}, cbi("cgminer/cgminer"), _("R1 Miner"),1)
   entry({"admin", "miner", "minerconfig"}, cbi("cgminer/cgminerconfig"), _("U3 Configuration"),2)
    entry({"admin", "miner", "minerstatus"}, cbi("cgminer/cgminerstatus"), _("U3 Status"),3)
   --entry({"admin", "status", "cgminerapi"}, call("action_cgminerapi"), _("Cgminer API Log"))
end

function action_cgminerapi()
   local pp   = io.popen("echo -n \"[Firmware Version] => \"; cat /etc/avalon_version; /usr/bin/cgminer-api stats;")
    local data = pp:read("*a")
    pp:close()

    luci.template.render("cgminerapi", {api=data})
end

function num_commas(n)
   return tostring(math.floor(n)):reverse():gsub("(%d%d%d)","%1,"):gsub(",(%-?)$","%1"):reverse()
end

whole_hw=0
function summary()
   local data = {}
   local summary = luci.util.execi("/usr/bin/cgminer-api -o summary | sed \"s/|/\\n/g\" ")

   if not summary then
      return
   end

   for line in summary do
      local elapsed, ghs5s, ghsav, foundblocks, getworks, accepted, rejected, hw, utility, discarded, stale, getfailures, localwork, remotefailures, networkblocks, totalmh, wu, diffaccepted, diffrejected, diffstale, bestshare = line:match("Elapsed=(%d+),GHS 5s=(.*),GHS av=(.*),Found Blocks=(%d+),Getworks=(%d+),Accepted=(%d+),Rejected=(%d+),Hardware Errors=(%d+),Utility=([%d%.]+),Discarded=(%d+),Stale=(%d+),Get Failures=(%d+),Local Work=(%d+),Remote Failures=(%d+),Network Blocks=(%d+),Total MH=([%d%.]+),Work Utility=([%d%.]+),Difficulty Accepted=([%d]+)%.%d+,Difficulty Rejected=([%d]+)%.%d+,Difficulty Stale=([%d]+)%.%d+,Best Share=(%d+)")
      if elapsed then
         local str
         local days
         local h
         local m
         local s = elapsed % 60;
         elapsed = elapsed - s
         elapsed = elapsed / 60
         if elapsed == 0 then
            str = string.format("%ds", s)
         else
            m = elapsed % 60;
            elapsed = elapsed - m
            elapsed = elapsed / 60
            if elapsed == 0 then
               str = string.format("%dm %ds", m, s);
            else
               h = elapsed % 24;
               elapsed = elapsed - h
               elapsed = elapsed / 24
               if elapsed == 0 then
                  str = string.format("%dh %dm %ds", h, m, s)
               else
                  str = string.format("%dd %dh %dm %ds", elapsed, h, m, s);
               end
            end
         end
         whole_hw=hw
         data[#data+1] = {
            ['elapsed'] = str,
            ['ghs5s'] = ghs5s,
            ['ghsav'] = ghsav,
            ['foundblocks'] = foundblocks,
            ['getworks'] = num_commas(getworks),
            ['accepted'] = num_commas(accepted),
            ['rejected'] = num_commas(rejected),
            ['hw'] = num_commas(hw),
            ['utility'] = num_commas(utility),
            ['discarded'] = num_commas(discarded),
            ['stale'] = stale,
            ['getfailures'] = getfailures,
            ['localwork'] = num_commas(localwork),
            ['remotefailures'] = remotefailures,
            ['networkblocks'] = networkblocks,
            ['totalmh'] = string.format("%e",totalmh),
            ['wu'] = num_commas(wu),
            ['diffaccepted'] = num_commas(diffaccepted),
            ['diffrejected'] = num_commas(diffrejected),
           -- ['diffstale'] = diffstale,
            ['bestshare'] = num_commas(bestshare)
         }
      end
   end
   return data
end


function blocks()

	luci.util.exec("sed -i -r -e '/^$/d' /etc/config/getblk");
	local file = io.open("/etc/config/getblk", "r");		
	local data = {};
	
	if not file then
		data[#data+1] = {['Comment'] = "You have not found blocks yet!"}
		return 0,data
	end
	local flag = 0
	for l in file:lines() do
			if l ~= "" then         
		  data[#data+1] = {
				  ['blockInfo'] = l		            
		   }
			 flag = 1
		end
	end
	file:close();

	if flag == 0 then
		data[#data+1] = {['Comment'] = "You have not found blocks yet!"}
		return 0,data
	end
	return 1,data;

end

function getGhs(ghs)
	if string.find(ghs,'K')~=nil then
	   ghs = string.gsub(ghs,"K","")/1000/1000
	elseif string.find(ghs,'M')~=nil then
	   ghs = string.gsub(ghs,"M","")/1000
	elseif string.find(ghs,'G')~=nil then
	   ghs = string.gsub(ghs,"G","")
	end
	return ghs
end

function summary_u3()
	local data = {}
	local ghs5s = 0
	local ghsav = 0
	local num = 0
	local stats = stats_u3()
	--io.popen("/bin/echo ".. stats.. " > /root/aa.txt")
 	if stats == nil then
	   data[#data+1] = {                                                                                                                     
	       ['num'] = num,                                                                                                                     
	       ['ghs5s'] = ghs5s,                                                                                                                 
               ['ghsav'] = ghsav                                                                                                                  
	   }                                                                                                                                    
	  return data 
	end
	if #stats ~=nil then
	   for i,v in pairs(stats) do
	     if type(v) == "table" then
	        local ghs_5s = getGhs(v['ghs5s'])
	        local ghs_av = getGhs(v['ghsav'])
	        ghs5s = ghs5s + ghs_5s
	        ghsav = ghsav + ghs_av
	     end
	   end
	   num = #stats
	   data[#data+1]={
	     ['num'] = string.format("%d", num),
	     ['ghs5s'] = string.format("%.3f" ,ghs5s),
	     ['ghsav'] = string.format("%.3f" ,ghsav)
	   }
	   return data
	 end
end

function stats_u3()
   local data = {}
   local status = luci.util.execi("/usr/bin/cgminer-api -o stats 127.0.0.1 4029 | sed \"s/|/\\n/g\" ")
   --local status = luci.util.execi("/bin/cat /root/a | sed \"s/|/\\n/g\" ")
   
   if not status then
      return
   end

   
   for line in status do
     local stats,id,elapsed,calls,wait,max,min,ghs5s,ghsav,read_time= line:match("STATS=(%d+),ID=(.*),Elapsed=(.*),Calls=(.*),Wait=(.*),Max=(.*),Min=(.*),GHS 5s=(.*),GHS av=(.*),read_time=(.*)")
     if elapsed then
         local str
         local days
         local h
         local m
         local s = elapsed % 60;
         elapsed = elapsed - s
         elapsed = elapsed / 60
         if elapsed == 0 then
            str = string.format("%ds", s)
         else
            m = elapsed % 60;
            elapsed = elapsed - m
            elapsed = elapsed / 60
            if elapsed == 0 then
               str = string.format("%dm %ds", m, s);
            else
               h = elapsed % 24;
               elapsed = elapsed - h
               elapsed = elapsed / 24
               if elapsed == 0 then
                  str = string.format("%dh %dm %ds", h, m, s)
               else
                  str = string.format("%dd %dh %dm %ds", elapsed, h, m, s);
               end
            end
         end

	 data[#data+1] = {
            ['stats'] = string.format("%d",stats),
            ['elapsed'] = str,
            ['ghs5s'] = string.format("%.3f",getGhs(ghs5s)),
            ['ghsav'] = string.format("%.3f",getGhs(ghsav)),
         }
    end
   end
   return data
end

function devs()
   local st, m5, fv
   local data = {}
   local fver = luci.util.exec("head -n1 /etc/avalon_version")
   local devs = luci.util.execi("/usr/bin/cgminer-api -o devs | sed \"s/|/\\n/g\";/usr/bin/cgminer-api -o stats | sed \"s/|/\\n/g\" ")

   if not devs then
      return
   end

   for line in devs do
      local fv = fver:match("(.*)")
      local status, mhs5s = line:match("Status=(%a+).*MHS 5s=([%d%.]+)")
      local mc, ac, f, vol, f1, f2, f3, f4, t1, t2, t3, t4, dh, nmw, cacn1, cacn2, cacn3, cacn4, cacs1, cacs2, cacs3, cacs4, cv = line:match("miner_count=(%d+),asic_count=(%d+),.*,frequency=(.*),voltage=(.*),.*,fan1=(%d+),fan2=(%d+),fan3=(%d+),fan4=(%d+),.*,temp1=(.*),temp2=(.*),temp3=(.*),temp4=(.*),.*,Device Hardware%%=([%d%.]+),no_matching_work=(%d+),chain_acn1=(%d+),chain_acn2=(%d+),chain_acn3=(%d+),chain_acn4=(%d+),.*,chain_acs1=(.*),chain_acs2=(.*),chain_acs3=(.*),chain_acs4=(.*),.*,USB Pipe=(%d+)")
      if mhs5s then
        st = status
        m5 = mhs5s
      end
      if mc then
        for i = 1, mc, 1 do
          if i == 1 then
            data[#data+1] = {
              ['chain'] = '1',
              ['fan_name'] = "Speed(r/min)",
              ['asic'] = cacn1,
              ['frequency'] = num_commas(f),
              ['v'] = vol,
              ['fan1'] = num_commas(f1),
              ['fan2'] = num_commas(f2),
              ['fan3'] = num_commas(f3),
              ['fan4'] = num_commas(f4),
              ['temp'] = t1,
              ['status'] = cacs1
            }
          elseif i == 2 then
            data[#data+1] = { 
              ['chain'] = '2',
              ['asic'] = cacn2,
              ['frequency'] =  num_commas(f),
              ['temp'] = t2,
              ['status'] = cacs2
            }
          elseif i == 3 then
            data[#data+1] = {
              ['chain'] = '3',
              ['asic'] = cacn3,
              ['frequency'] =  num_commas(f),
              ['temp'] = t3,
              ['status'] = cacs3
            } 
          elseif i == 4 then  
            data[#data+1] = {
              ['chain'] = '4',
              ['asic'] = cacn4,
              ['frequency'] =  num_commas(f),
              ['temp'] = t4,
              ['status'] = cacs4
            }
          end
        end
      end
   end

   return data
end


function chains()
   local st, m5, fv
   local data = {}
   local fver = luci.util.exec("head -n1 /etc/avalon_version")
   local chains = luci.util.execi("/usr/bin/cgminer-api -o devs | sed \"s/|/\\n/g\";/usr/bin/cgminer-api -o stats | sed \"s/|/\\n/g\" ")

   if not chains then
      return
   end

   for line in chains do
      local fv = fver:match("(.*)")
      local status, mhs5s = line:match("Status=(%a+).*MHS 5s=([%d%.]+)")
      local mc, ac, f, f1, f2, f3, t1, t2, t3, dh, nmw, cv = line:match("miner_count=(%d+),asic_count=(%d+),.*,frequency=(%d+),.*,fan1=(%d+),fan2=(%d+),fan3=(%d+),.*,temp1=([%-%d]+),temp2=([%-%d]+),temp3=([%-%d]+),.*Device Hardware%%=([%d%.]+),no_matching_work=(%d+),.*,USB Pipe=(%d+)")
      if mhs5s then
	 st = status
	 m5 = mhs5s
      end
      if mc then
	 data['only'] = {
	    ['status'] = st,
	    ['mhs5s'] = m5,
	    ['frequency'] = f,
	    ['minercount'] = mc,
	    ['asiccount'] = ac,
	    ['fan1'] = f1,
	    ['fan2'] = f2,
	    ['fan3'] = f3,
	    ['temp1'] = t1,
	    ['temp2'] = t2,
	    ['temp3'] = t3,
	    ['dh'] = dh,
	    ['nmw'] = nmw,
	    ['fv'] = fv,
	    ['cv'] = cv
	 }
      end
   end

   return data
end



function pools(mtype)
--  local sum_gw,sum_a,sum_da,sum_ds,sum_dr,sum_dsta,sum_r,sum_dc,sum_sta
   local data = {}
   local pools
   if mtype == "r1" then
	pools = luci.util.execi("/usr/bin/cgminer-api -o pools | sed \"s/|/\\n/g\" ")
   elseif mtype=="u3" then
        pools = luci.util.execi("/usr/bin/cgminer-api -o pools 127.0.0.1 4029 | sed \"s/|/\\n/g\" ")
   end
   sum_gw=0 sum_a=0 sum_da=0 sum_ds=0 sum_dr=0 sum_dsta=0 sum_r=0 sum_dc=0 sum_sta=0
   if not pools then
      return
   end

   for line in pools do
      local pi, url, st, pri, lp, gw, a, r, dc, sta, gf, rf, user, lst, di, ds, da, dr, dsta, lsd, hs, sa, su, hg = line:match(
         "POOL=(%d+),URL=(.*),Status=(%a+),Priority=(%d+),.*,Long Poll=(%a+),Getworks=(%d+),Accepted=(%d+),Rejected=(%d+),Discarded=(%d+),Stale=(%d+),Get Failures=(%d+),Remote Failures=(%d+),User=(.*),Last Share Time=(.*),Diff=(.*),Diff1 Shares=(%d+),Proxy Type=.*,Proxy=.*,Difficulty Accepted=(%d+)[%.%d]+,Difficulty Rejected=(%d+)[%.%d]+,Difficulty Stale=(%d+)[%.%d]+,Last Share Difficulty=(%d+)[%.%d]+,Has Stratum=(%a+),Stratum Active=(%a+),Stratum URL=.*,Has GBT=(%a+)")
      if pi then
         if lst == "0" then
            lst_date = "Never"
         else
            --lst_date = os.date("%c", lst)
            lst_date = lst
         end
         sum_gw=sum_gw+gw
         sum_a=sum_a+a
         sum_da=sum_da+da
         sum_ds=sum_ds+ds
         sum_dr=sum_dr+dr
         sum_dsta=sum_dsta+dsta
         sum_r=sum_r+r
         sum_dc=sum_dc+dc 
         sum_sta=sum_sta+sta
         data[#data+1] = {
            ['pool'] = pi, 
            ['url'] = url,
            ['user'] = user,
            ['status'] = st,
            ['priority'] = pri,
            ['getworks'] =  num_commas(gw),
            ['accepted'] = num_commas(a),
            ['rejected'] =num_commas(r),
            ['discarded'] = num_commas(dc), 
            ['stale'] = sta,
            ['diff'] = di,
            ['diff1shares'] =num_commas(ds) , 
            ['diffaccepted'] = num_commas(da),
            ['diffrejected'] = num_commas(dr),
            ['diffstale'] =  num_commas(dsta), 
            ['lastsharedifficulty'] = num_commas(lsd), 
            ['lastsharetime'] = lst_date,
            ['hasstratum'] = hs,
            ['stratumactive'] = sa,
            ['stratumurl'] = su,
            ['hasgbt'] = hg
         }
      end
   end
----------------------------------------------
----------------total------------------------
--[[
gw="%d"
a="%d" 
da="%d" 
ds="%d" 
dr="%d"
dsta="%d"
r="%d"
dc="%d"
sta="%d"
gw=gw:format(sum_gw)
a=a:format(sum_a) 
da=da:format(sum_da) 
ds=ds:format(sum_ds) 
dr=dr:format(sum_dr) 
dsta=dsta:format(sum_dsta) 
r=r:format(sum_r) 
dc=dc:format(sum_dc) 
sta=sta:format(sum_sta) 
 data[#data+1] = {
	['pool'] = "total",                                                                                                         
	['url'] = url,                                                                                                          
	['user'] = user,                                                                                                       
	['status'] = st,                                                                                                       
	['priority'] = pri,                                                                                                     
	['getworks'] =  num_commas(gw),                                                                                                     
	['accepted'] =  num_commas(a),                                                                                                      
	['rejected'] = r,                                                                                                       
	['discarded'] = dc,                                                                                                    
	['stale'] = num_commas(sta),                                                                                                       
	['diff'] = di,                                                                                                         
	['diff1shares'] =num_commas(ds) ,                                                                                                   
	['diffaccepted'] = num_commas(da),         
	['diffrejected'] = num_commas(dr),                                                      
	['diffstale'] = num_commas(dsta),                                                            
	['lastsharedifficulty'] = lsd,                                                    
	['lastsharetime'] = nil,  
	['hasstratum'] = hs,          
	['stratumactive'] = sa,       
	['stratumurl'] = su,                                                                                    
	['hasgbt'] = hg  
 }
--------------------------------
-------------HW----------------- attempt to perform arithmetic on global 'whole_hw' (a nil value)
hw_str="%d"
hw_diff1_str="%.4f%%"
hw_diffA_str="%.4f%%"
hw_diff1=(whole_hw/sum_ds)*100
hw_diffA=(whole_hw/sum_da)*100
hw_str=hw_str:format(whole_hw)
hw_diff1_str=hw_diff1_str:format(hw_diff1)
hw_diffA_str=hw_diffA_str:format(hw_diffA)
data[#data+1] = {
	['pool'] = "HW",                                                                                                         
	['url'] = hw_str ,                                                                                                         
	['user'] = nil,                                                                                                       
	['status'] = nil,                                                                                                       
	['priority'] = pri,                                                                                                    
	['getworks'] = nil,                                                                                                     
	['accepted'] = nil,                                                                                                      
	['rejected'] = nil,                                                                                                      
	['discarded'] = nil,                                                                                                    
	['stale'] = nil,                                                                                                       
	['diff'] = nil,                                                                                                         
	['diff1shares'] = hw_diff1_str,                                                                                                   
	['diffaccepted'] = hw_diffA_str,         
	['diffrejected'] = nil,                                                      
	['diffstale'] = nil,                                                            
	['lastsharedifficulty'] = nil,                                                    
	['lastsharetime'] = nil,  
	['hasstratum'] = nil,          
	['stratumactive'] = nil,       
	['stratumurl'] = nil,                                                                                    
	['hasgbt'] = nil     
                                                                                          
                                                                                          
} 
--]]
   return data
end