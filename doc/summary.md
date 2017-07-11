### Summary of results

#### UKF Roadmap

<image src="UKF_roadmap.png" alt="missing UKF Roadmap image" />
<a> (Image Source: Udacity classroom lectures)</a>

#### RMSE values before tuning of process noise params (std_a_=30; std_yawdd_ = 30)
<table>
<tr>
<th>RMSE attribute</th>
<th>Lidar only</th>
<th>Radar only</th>
<th>Both</th>
</tr>
<tr>
<td>X</td>
<td>.1974</td>
<td>.5549</td>
<td>.0976</td>
</tr>
<tr>
<td>Y</td>
<td>.1791</td>
<td>1.8698</td>
<td>.1210</td>
</tr>
<tr>
<td>VX</td>
<td>1.6911</td>
<td>4.6186</td>
<td>.8693</td>
</tr>
<tr>
<td>VY</td>
<td>1.7518</td>
<td>5.2593</td>
<td>.9820</td>
</tr>
</table>

As we see in the table above the RMSE values are clearly too high, for the given process noise parameters. We need to tune them. And we use NIS for it.

<image src="nis.png" alt="missing NIS image" />
<a> (Image Source: Udacity classroom lectures)</a>

#### RMSE Values after tuning of process noise values

<table>
<tr>
<th>RMSE attribute</th>
<th>Lidar only</th>
<th>Radar only</th>
<th>Both</th>
</tr>
<tr>
<td>X</td>
<td>.1840</td>
<td>.2333</td>
<td>.0963</td>
</tr>
<tr>
<td>Y</td>
<td>.1543</td>
<td>.3183</td>
<td>.0854</td>
</tr>
<tr>
<td>VX</td>
<td>.6056</td>
<td>.5284</td>
<td>.4136</td>
</tr>
<tr>
<td>VY</td>
<td>.4862</td>
<td>.7036</td>
<td>.4807</td>
</tr>
</table>
