<h1>Files from Scott:</h1>
<p>These files are sent from Scott @ iInvent prior to sending the prototypes

They included 4 main units, 1 ESP01 relay, 1 Sonoff Relay, and 1 Tasmota-compatible smart plug.</p>
<hr>
<h2>LED dictionary</h2>
<h3>Main Unit</h3>
<table>
    <tr>
        <th>LED action</th>
        <th>Description</th>
    </tr>
    <tr>
        <td>Blue OFF: <br>flashes every few seconds</td>
        <td>Normal Mode: Power is ON according to sensor</td>
    </tr>
    <tr>
        <td>Blue ON: <br>flashes off every few seconds</td>
        <td>Normal Mode: Power is OFF according to sensor
    <tr>
        <td>All 3 blink twice,<br> then Blue for 3 seconds</td>
        <td>The device is in <b>Setup Mode</b> after a startup</td>
    </tr>
    <tr>
        <td>Blue rapid flash</td>
        <td>Hub is in pairing mode</td>
    </tr>
    <tr>
        <td>All 3 light in squence</td>
        <td>Hub is in clear mode, deleting switches</td>
    </tr>
    <tr>
        <td>n Green blink(s)</td>
        <td>Indicating number of switches paired and online</td>
    </tr>
    <tr>
        <td>n Red blink(s)</td>
        <td>indicating number of paired switches but offline</td>
    </tr>
</table>
<h3>ESP01 Switch</h3>
<table>
    <tr>
        <th>LED action</th>
        <th>Description</th>
    </tr>
    <tr>
        <td>Steady light</td>
        <td>Setup mode time to press the pairing <br>button if pairing is needed</td>
    </tr>
    <tr>
        <td>Rapid flash</td>
        <td>Pairing mode</td>
</table>
<hr>
<h2>Pairing:</h2>
<h3>Main Unit:</h3>
<p>The main unit piars by pressing the button for t seconds (where 0.5s &lt t &lt 2s).<br>When this is pressed, the main unit enters <i>Pairing</i> mode for 30 seconds. During that, the blue LED flashes quickly.During the <i>pairing</i> mode, the hub will add any switch that is also in pairing mode.<br>
If the button is pressed for more than 2 seconds, it will enter <i>clear</i> mode, erasing all paired switches. During that mode, all 3 LEDs will light up in sequence.</p>
<h3>ESP01 Switches:</h3>
<p>Button can be pressed to initiate pairing. This happens when the device is starting up. When it reaches the steady LED On after a reboot, the button can be pressed and the device will set a flag on EEPROM for <i>pairing</i> mode. When the restart is complete, the device gets into mode in which durin gthe LED will flash rapidly. When successful, the switch will reset into normal mode. If no pairing is successful, the device will return to normal mode.
</P>




