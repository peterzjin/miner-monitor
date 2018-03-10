#!/usr/bin/python
import rrdtool
import datetime

def miner_type(n):
    return {
            0: 'CM_ETH_D',
            1: 'CM_ETH',
            2: 'CM_ZEC',
            3: 'ZM_ZEC',
        }.get(int(n),'error')

def efficiency(acp, rej, inc):
    sum_s = acp + rej + inc
    if sum_s == 0:
        return "n/a"
    return "%.2f%%" % (acp * 100 / sum_s)

def sec2time(val):
    tval = int(val)
    tsec = val % 60
    tval = tval / 60
    tmin = val % 60
    tval = tval / 60
    thur = val % 24
    tday = tval / 24
    t = ""
    if tday != 0:
        t += "%dD" % tday
    if thur != 0:
        t += "%dH" & thur
    if tmin != 0:
        t += "%dD" % tmin
    if tsec != 0:
        t += "%dH" & tsec
    if t == "":
        t = "0"
    return t

lastupdate = rrdtool.lastupdate("../miner_state.rrd")
ds = lastupdate['ds']
miners = int(ds['miners'])
gpus = int(ds['gpus'])
lu_date = lastupdate['date']
now_date = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

print '<html>'
print '<head>'
print '<title>Miner State</title>'
print '<link rel="stylesheet" href="miner.css" type="text/css" />'
print '</head>'
print '<meta http-equiv="refresh" content="360">'
print '<body>'
print '<table class="flatTable">'
print '<tr class="titleTr">'
print '<td width="100" colspan="6"><img src="mining.png"/></td>'
print '<td colspan="4"><p>Water: %.2f &#176;c | Inlet: %.2f &#176;c | Outlet: %.2f &#176;c | PWM: %.0f</p>Updated at %s | Now %s</td>' % (ds['s_temp0'], ds['s_temp2'], ds['s_temp1'], ds['pwm0'], lu_date, now_date)
print '</tr>'
print '<tr class="headingTr">'
print '<td>Miner</td>'
print '<td>Status</td>'
print '<td>GPUs</td>'
print '<td>Type</td>'
print '<td>Rate</td>'
print '<td>Temp</td>'
print '<td>Shares(A/R/I)</td>'
print '<td>Efficiency</td>'
print '<td>Uptime</td>'
print '<td>Offtime</td>'
print '</tr>'
l_gpus = 0
for i in range(miners):
    m_gpus = int(ds['m_gpus' + str(i)])
    print '<tr>'
    print '<td>#%d</td>' % i
    if m_gpus == 0:
        print '<td>Offline</td>'
    else:
        print '<td>Online</td>'
    print '<td>%d</td>' % m_gpus
    print '<td>%s</td>' % miner_type(ds['m_type' + str(i)])
    if ds['m_type' + str(i)] == 0:
        print '<td>%.2f MH/s | %.2f MH/s</td>' % (ds['m_hash' + str(i)], ds['m_dhash' + str(i)])
    else:
        print '<td>%.2f MH/s</td>' % ds['m_hash' + str(i)]
    print '<td>%.2f &#176;c</td>' % ds['m_temp' + str(i)]
    print '<td>%.0f/%.0f/%.0f</td>' % (ds['m_acp_s' + str(i)], ds['m_rej_s' + str(i)], ds['m_inc_s' + str(i)])
    print '<td>%s</td>' % efficiency(ds['m_acp_s' + str(i)], ds['m_rej_s' + str(i)], ds['m_inc_s' + str(i)])
    print '<td>%s</td>' % datetime.timedelta(minutes = ds['m_uptime' + str(i)])
    print '<td>%s</td>' % datetime.timedelta(seconds = ds['m_offtime' + str(i)])
    print '</tr>'

    if m_gpus != 0:
        print '<tr>'
        print '<td></td>'
        print '<td></td>'
        print '<td colspan="8" align="center">'
        print '<table class="flatTable_g">'
        print '<tr class="headingTr">'
        print '<td>GPU</td>'
        print '<td>Rate</td>'
        print '<td>Temp/Fan</td>'
        print '<td>Shares(A/R/I)</td>'
        print '<td>Efficiency</td>'
        print '</tr>'
        for j in range(l_gpus, l_gpus + m_gpus):
            print '<tr>'
            print '<td>#%d</td>' % (j - l_gpus)
            if ds['m_type' + str(i)] == 0:
                print '<td>%.2f MH/s | %.2f MH/s</td>' % (ds['g_hash' + str(j)], ds['g_dhash' + str(j)])
            else:
                print '<td>%.2f MH/s</td>' % ds['g_hash' + str(j)] 
            print '<td>%.2f &#176;c / %d%%</td>' % (ds['g_temp' + str(j)], ds['g_fan' + str(j)])
            print '<td>%.0f/%.0f/%.0f</td>' % (ds['g_acp_s' + str(j)], ds['g_rej_s' + str(j)], ds['g_inc_s' + str(j)])
            print '<td>%s</td>' % efficiency(ds['g_acp_s' + str(j)], ds['g_rej_s' + str(j)], ds['g_inc_s' + str(j)])
            print '</tr>'
        l_gpus += m_gpus
        print '</table>'
        print '</td>'
        print '</tr>'

print '</table>'
print '</body>'
print '</html>'
