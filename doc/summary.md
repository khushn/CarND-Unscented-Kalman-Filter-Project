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

We use the following NIS stastical chi square distribution to compute NIS. 

<image src="nis_chi_squared_dstbn.png" alt="missing NIS chi squared distribution" />
<a> (Image Source: Udacity classroom lectures)</a>

##### NIS distribution for the current values

###### Laser NIS distribution (Using threshold 5.991)
So clearly we have scope to reduced the process noise parameters
<image src="laser_nis_distbn.png" alt="missing Laser NIS" />

###### Radar NIS distribution (Using threshold 7.815)
In this case we don't have as much leeway to reduce process noise parameters. 

<image src="radar_nis_distbn.png" alt="missing Radar NIS" />

But we need to take into account the combined effect of Lidar and Radar. So we can clearly reduce process noise. 
Moreover, the values (std_a_=30 and std_yawdd_=30) are too high for cycle. So we experiement and record the results with the ideal (or at least better) values of process noise.

#### RMSE Values at various process noise values 
<table>
<tr>
<th/>
<th>std_a=30; std_yaw=30</th>
<th>std_a=6; std_yaw=3</th>
<th>std_a=3; std_yaw=1.5</th>
<th>std_a=1.5; std_yaw=1.5</th>
<th>std_a=1; std_yaw=1 (best)</th>
<th>std_a=0; std_yaw=0</th>
</tr>
<tr>
<td>X</td>
<td>..0976</td>
<td>.08</td>
<td>.0736</td>
<td>.0678</td>
<td><b>.0647</b></td>
<td>.13.97</td>
</tr>
<tr>
<td>Y</td>
<td>.1210</td>
<td>.0942</td>
<td>.0867</td>
<td>.0845</td>
<td><b>.0838</b></td>
<td>5.456</td>
</tr>
<tr>
<td>VX</td>
<td>.8693</td>
<td>.4202</td>
<td>.3593</td>
<td>.3415</td>
<td><b>.3313</b></td>
<td>4.2161</td>
</tr>
<tr>
<td>VY</td>
<td>.9820</td>
<td>.3562</td>
<td>.2702</td>
<td>.2490</td>
<td><b>.2327</b></td>
<td>1.5236</td>
</tr>
</table>

#### Final complete viualization

The final visualization at noise values (std_a_=1; std_yawdd_=1) is given below
<image src="final_complete_visual.png" alt="missing final complete visualization" />

The full visualization HTML using the nice tools provided by <a href="https://github.com/udacity/CarND-Mercedes-SF-Utilities"> Mercedes </a> is given <a href="/ukf-visualization-extended.html"> here </a>

#### Optional project of catching Runaway robot/Car

I was able to do this as well. The robot was always getting caught in just around 6 seconds with a very optimal effort from the hunter car!

The below image shows the screenshot of one such run. 

<image src="success_caught_the_robot.png" alt="missing screen grab of run away robot capture"/>

The link to the project which has the code for the Runaway Car is below: 
https://github.com/khushn/CarND-Catch-Run-Away-Car-UKF