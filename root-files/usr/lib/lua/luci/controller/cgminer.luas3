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
   entry({"admin", "status", "miner"}, cbi("cgminer/cgminer"), _("Miner Configuration"))
   entry({"admin", "status", "minerstatus"}, cbi("cgminer/cgminerstatus"), _("Miner Status"))
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
	    ['diffstale'] = diffstale,
	    ['bestshare'] = num_commas(bestshare)
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
      local mc, ac, f, vol, f1, f2, f3, f4, t1, t2, t3, t4, dh, nmw, cacn1, cacn2, cacn3, cacn4, cacs1, cacs2, cacs3, cacs4, cv = line:match("miner_count=(%d+),asic_count=(%d+),.*,frequency=(.*),voltage=(%d+),.*,fan1=(%d+),fan2=(%d+),fan3=(%d+),fan4=(%d+),.*,temp1=(.*),temp2=(.*),temp3=(.*),temp4=(.*),.*,Device Hardware%%=([%d%.]+),no_matching_work=(%d+),chain_acn1=(%d+),chain_acn2=(%d+),chain_acn3=(%d+),chain_acn4=(%d+),.*,chain_acs1=(.*),chain_acs2=(.*),chain_acs3=(.*),chain_acs4=(.*),.*,USB Pipe=(%d+)")
      if mhs5s then
        st = status
        m5 = mhs5s
      end
      if mc then
        for i = 1, mc, 1 do
          if i == 1 then
            data[#data+1] = {
              ['chain'] = '1',
              ['asic'] = cacn1,
              ['frequency'] = f,
              ['fan'] = f1,
              ['temp'] = t1,
              ['status'] = cacs1
            }
          elseif i == 2 then
            data[#data+1] = {
              ['chain'] = '2',
              ['asic'] = cacn2,
              ['frequency'] = f,
              ['fan'] = f2,
              ['temp'] = t2,
              ['status'] = cacs2
            }
          elseif i == 3 then
            data[#data+1] = {
              ['chain'] = '3',
              ['asic'] = cacn3,
              ['frequency'] = f,
              ['fan'] = f3,
              ['temp'] = t3,
              ['status'] = cacs3
            }
          elseif i == 4 then
            data[#data+1] = {
              ['chain'] = '4',
              ['asic'] = cacn4,
              ['frequency'] = f,
              ['fan'] = f4,
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

function pools()
   local data = {}
   local pools = luci.util.execi("/usr/bin/cgminer-api -o pools | sed \"s/|/\\n/g\" ")

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
	 data[#data+1] = {
	    ['pool'] = pi,
	    ['url'] = url,
	    ['user'] = user,
	    ['status'] = st,
	    ['priority'] = pri,
	    ['getworks'] = gw,
	    ['accepted'] = a,
	    ['rejected'] = r,
	    ['discarded'] = dc,
	    ['stale'] = sta,
	    ['diff'] = di,
	    ['diff1shares'] = ds,
	    ['diffaccepted'] = da,
	    ['diffrejected'] = dr,
	    ['diffstale'] = dsta,
	    ['lastsharedifficulty'] = lsd,
	    ['lastsharetime'] = lst_date,
	    ['hasstratum'] = hs,
	    ['stratumactive'] = sa,
	    ['stratumurl'] = su,
	    ['hasgbt'] = hg
	 }
      end
   end

   return data
end
