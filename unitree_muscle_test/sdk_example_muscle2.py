#!/usr/bin/python

import sys
import time
import math
import numpy as np
import pandas as pd

sys.path.append('./lib/python/arm64')
import robot_interface as sdk

def trap_wave(x,a,m,l,c):
  return a/np.pi*(np.arcsin(np.sin((np.pi/m)*x+l))+np.arccos(np.cos((np.pi/m)*x+l)))+c
class muscle_pair:
	def __init__(self):
		self.ml = np.zeros((2,))
		self.mv = np.zeros((2,))
		self.k_al = np.array([0.02/2. * 180/np.pi]*2)
		self.qn = 0.0
		self.Fl = np.zeros((2,))
		self.Fv = np.zeros((2,))
		self.Fp = np.zeros((2,))
		self.FMax = np.array([16,16])
		self.F = np.zeros((2,))
		self.ma = np.array([0.25,0.25])
		self.torques = np.zeros((2,))
		self.act = np.zeros((2,))
		self.dact = np.zeros((2,))
		self.tau_act = 0.02
		self.tau_deact = 0.02
		self.dt = 0.001
		self.l0 = 0.85


	def step(self,q,dq,input):
		self.dact = - (self.tau_act / self.tau_deact + (1.0 - self.tau_act / self.tau_deact) * input) * self.act / self.tau_act + input / self.tau_act
		self.act += self.dact*self.dt
		deltaQ = q-self.qn 

		self.ml[0] = self.l0 * (1.0 - self.k_al[0] * deltaQ)
		self.ml[1] = self.l0 * (1.0 + self.k_al[1] * deltaQ)
		self.mv[0] = -self.l0 * self.k_al[0] * dq * 0.075
		self.mv[1] = self.l0 * self.k_al[1] * dq * 0.075

		
		self.Fl = np.exp(-np.power(np.abs((np.power(self.ml, 2.3) - 1.0) / 1.26), 1.62))

		for i in range(2):
			if self.mv[i]<0.:
				self.Fv[i] = (- 0.69 - 0.17 * self.mv[i]) / (self.mv[i] - 0.69)
			else:
				tmp = - 5.34 * self.ml[i] * self.ml[i] + 8.41 * self.ml[i] - 4.7;
				self.Fv[i] = (0.18 - tmp * self.mv[i]) / (self.mv[i] + 0.18)
		#self.Fv=(np.exp(self.mv)/(np.exp(self.mv)+1))+0.5
		
		self.Fp = 70.0 * 0.045 * np.log(np.exp((self.ml - 1.4) / 0.05) + 1.0) - 0.018 * (np.exp(-18.7 * (self.ml - 0.79)) - 1.0)

		self.F = self.FMax * (self.act * self.Fl*self.Fv + self.Fp)
		self.F[self.F<0] = 0
		self.F[self.F>self.FMax] = self.FMax[0]
		self.torques = self.F * self.ma
		return self.torques[0]-self.torques[1]

if __name__ == '__main__':

	d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
		 'FL_0':3, 'FL_1':4, 'FL_2':5, 
		 'RR_0':6, 'RR_1':7, 'RR_2':8, 
		 'RL_0':9, 'RL_1':10, 'RL_2':11 }
	PosStopF  = math.pow(10,9)
	VelStopF  = 16000.0
	HIGHLEVEL = 0xee
	LOWLEVEL  = 0xff
	dt = 0.001
	fc=0.1/dt
	alpha = (fc*np.pi*2)/(fc*np.pi*2+1)

	sin_count = 0
	mp = muscle_pair()
	input_ = np.array([0.,0.])
	udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
	safe = sdk.Safety(sdk.LeggedType.Go1)
	
	cmd = sdk.LowCmd()
	state = sdk.LowState()
	udp.InitCmdData(cmd)

	Tpi = 0
	motiontime = 0


	out = np.zeros((11,40000))
	#import IPython;IPython.embed()
	#print(cmd.motorCmd[d['FR_2']])
	
	while True:
		ti0 = time.time()
		motiontime += 1
		time_= motiontime*dt
		# freq_Hz = 1
		freq_Hz = 2
		# freq_Hz = 5;
		freq_rad = freq_Hz * 2* math.pi
		# t = dt*sin_count

		# print(motiontime)
		# print(state.imu.rpy[0])
		

		udp.Recv()
		udp.GetRecv(state)
		if motiontime ==1:
			dqf = state.motorState[d['FR_1']].dq

		dqf = dqf +alpha*(state.motorState[d['FR_1']].dq-dqf)

		if( motiontime >= 500):
			#input_[0]=(1+np.sin(time_))
			#input_[1]=(1+np.sin(time_+np.pi))
			#input_[0]=np.max([np.sin(time_/2),0])*0.4
			#input_[1]=np.max([np.sin(time_/2+np.pi),0])*0.4
			#input_[input_>0.]=.2
			#input_+=0.2
			#input_[1]=0.2

			period = 5
			a_h = 0.6
			a_l = 0.1
			input_[0]=trap_wave(time_,a_h-a_l,period,0,0)+a_l
			input_[1]=-trap_wave(time_,a_h-a_l,period,0,0)+a_h
			#if time_<=10.:
			#	input_[0]=input_[1]=0
			#elif time_<=15.:
			#	input_[0]=input_[1]=0.125
			#elif time_<=20.:
			#	input_[0]=0.125
			#	input_[1]=0.25

			dq = state.motorState[d['FR_1']].dq
			
			#dq = dqf
			torque =  mp.step(state.motorState[d['FR_1']].q,dq,input_)
			
			if time_ < 5.0:
				torque *= time_/5.0
				#print(time_/10.0)
			
			#torque = (0 - state.motorState[d['FR_1']].q)*10.0 + (0 - state.motorState[d['FR_1']].dq)*1.0
			# torque = (0 - state.motorState[d['FR_1']].q)*20.0 + (0 - state.motorState[d['FR_1']].dq)*2.0
			# torque = 2 * sin(t*freq_rad)
			torque = np.fmin(np.fmax(torque, -5.0), 5.0)
			# torque = np.fmin(np.fmax(torque, -15.0), 15.0)
			#print(torque)

			cmd.motorCmd[d['FR_1']].q = PosStopF
			cmd.motorCmd[d['FR_1']].dq = VelStopF
			cmd.motorCmd[d['FR_1']].Kp = 0
			cmd.motorCmd[d['FR_1']].Kd = 0
			cmd.motorCmd[d['FR_1']].tau = torque

			if motiontime < 40500:
				out[:,motiontime-500] = np.array([input_[0],
												  input_[1],
												  mp.act[0],mp.act[1],
												  torque,
												  state.motorState[d['FR_1']].q,
												  state.motorState[d['FR_1']].dq,
												  state.motorState[d['FR_1']].ddq,
												  state.motorState[d['FR_1']].tauEst,
												  state.motorState[d['FR_1']].temperature,
												  dqf])
			if motiontime == 40500:
				df=pd.DataFrame(out.T,columns=['input1','input2','act1','act2','torque','q','dq','ddq','tauEst','temperature','dqf'])
				df.to_csv('test.csv')

		if(motiontime > 10):
			safe.PowerProtect(cmd, state, 1)
		
		if(motiontime > 100):
			cmd.motorCmd[d['FR_0']].tau = -0.65
			cmd.motorCmd[d['FL_0']].tau = 0.65
			cmd.motorCmd[d['RR_0']].tau = -0.65
			cmd.motorCmd[d['FL_0']].tau = 0.65

		udp.SetSend(cmd)
		udp.Send()
		
		print(time.time()-ti0)
		if (time.time()-ti0)<dt:
			time.sleep(dt-(time.time()-ti0))
			print(dt)