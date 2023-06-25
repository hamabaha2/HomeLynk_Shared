<h1>Main unit software structure</h1>
<p>This document describes structure of the software</p>
<ul>Function structure
    <li>Setup:</li>
    <li>loop:</li>
    <li>Wifi: Handling Wifi
    <ul> Different Wifi Functions:
        <li>Wifi mode
        <li>Client Setup
        <li>AP setup
    </ul>
    <li>Pairing: Pairing logic [ESP echange, array update, EPROM handling]</li>
    <li>SetupMode: Temporary Wifi Settings, Webserver setup, Page Digest, Update EPROM</li>
    <li>ESPmessage:<ul>
        <li>Periodc command Messages</li>
        <li>Pairing Messages</li>
    </ul>
    </li>
    <li>EPROMcontrol: Read and save EPROM</li>
    <li>LEDcontrol: Change LED accordig to input variables</li>
    <li>BTNcontrol: Call functions based on button press</li>
    <li>Sensing: Reading and interpreting values</li>
    <li>FirmwareUpdate:</li>
    <li>APIcontrol:
        <ul>
        <li>Authentication and session control</li>
        <li>Pull</li>
        <li>push</li>
        </ul>
    </li>
</ul>