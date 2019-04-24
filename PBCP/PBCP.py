from abaqus import *
from abaqusConstants import *
import __main__
import math
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
import time
import os
import sys
import ctypes
import multiprocessing

def decorate(model_name, instance_name, x2d_axis, y2d_axis, x3d_axis, y3d_axis, z3d_axis, x2d_displacement, y2d_displacement, x3d_displacement, y3d_displacement, z3d_displacement):
	for T in (range(1)):
		modelName = model_name
		instanceName = instance_name
		mesherror=0.000000001
		error = False
		try:
			a = mdb.models[modelName].rootAssembly
		except:
			print("Name of a model is incorrect")
			error1 = 0
			message1 = ctypes.windll.user32.MessageBoxA
			returnValue = message1(error1, 'Name of a model is incorrect','Error', 0x30 | 0x0)
			continue

		try:
			Nodeset = mdb.models[modelName].rootAssembly.instances[instanceName].nodes
		except:
			print("Name of an instance is incorrect")
			error2 = 0
			error = True
			message2 = ctypes.windll.user32.MessageBoxA
			message2(error2, 'Name of an instance is incorrect','Error', 0x30 | 0x0)
			continue

		j = 0
		x = []
		y = []
		z = []
		for i in Nodeset:
			x.insert(j, i.coordinates[0])
			y.insert(j, i.coordinates[1])
			z.insert(j, i.coordinates[2])
			j = j + 1

		Max = max(x)
		May = max(y)
		Maz = max(z)
		Mnx = min(x)
		Mny = min(y)
		Mnz = min(z)

		if (Maz - Mnz)==0:
			for i in a.features.keys():
				if i.startswith('DN'):
					del a.features['%s' % (i)]
			a.ReferencePoint(point=(2, May-0.5*(May-Mny), 0))
			a.ReferencePoint(point=(1, May-0.5*(May-Mny), 0))
			a.ReferencePoint(point=(0, May-0.5*(May-Mny), 0))

			r1 = a.referencePoints

			d=3
			for i in r1.keys():
				refPoints1=(r1[i], )
				a.Set(referencePoints=refPoints1, name='DN%s' % (d))
				d=d-1

			## Identifying boundary nodes ##
			c1 = []
			c2 = []
			c3 = []
			c4 = []
			xfront = []
			xback = []
			yfront = []
			yback = []
			xfrontxyz = {}
			xbackxyz = {}
			yfrontxyz = {}
			ybackxyz = {}

			for i in Nodeset:
				if (Mnx) < i.coordinates[0] < (Max) and (Mny) < i.coordinates[1] < (May):
					continue

				if abs(i.coordinates[0]-Max)<=0 and abs(i.coordinates[1]-May)<=0:
					c1.insert(0,i.label)
				if abs(i.coordinates[0]-Mnx)<=0 and abs(i.coordinates[1]-May)<=0:
					c2.insert(0,i.label)
				if abs(i.coordinates[0]-Max)<=0 and abs(i.coordinates[1]-Mny)<=0:
					c3.insert(0,i.label)
				if abs(i.coordinates[0]-Mnx)<=0 and abs(i.coordinates[1]-Mny)<=0:
					c4.insert(0,i.label)
				if abs(i.coordinates[0]-Max)<=0 and abs(i.coordinates[1]-May)>0 and abs(i.coordinates[1]-Mny)>0:
					xfrontxyz[i.label]=[i.coordinates[0], i.coordinates[1]]
				if abs(i.coordinates[0]-Mnx)<=0 and abs(i.coordinates[1]-May)>0 and abs(i.coordinates[1]-Mny)>0:
					xbackxyz[i.label]=[i.coordinates[0], i.coordinates[1]]
				if abs(i.coordinates[1]-May)<=0 and abs(i.coordinates[0]-Max)>0 and abs(i.coordinates[0]-Mnx)>0:
					yfrontxyz[i.label]=[i.coordinates[0], i.coordinates[1]]
				if abs(i.coordinates[1]-Mny)<=0 and abs(i.coordinates[0]-Max)>0 and abs(i.coordinates[0]-Mnx)>0:
					ybackxyz[i.label]=[i.coordinates[0], i.coordinates[1]]



			for i in xfrontxyz.keys():
					for k in xbackxyz.keys():
							if abs(xfrontxyz[i][1] - xbackxyz[k][1])<=0:
									xfront.append(i)
									xback.append(k)


			for i in yfrontxyz.keys():
				for k in ybackxyz.keys():
					if abs(yfrontxyz[i][0] - ybackxyz[k][0]) <=0:
						yfront.append(i)
						yback.append(k)


			a = mdb.models[modelName].rootAssembly
			Nodeset = mdb.models[modelName].rootAssembly.instances[instanceName].nodes
			mdb.models[modelName].StaticStep(name='Step-1', previous='Initial')


			## Sets ##
			a.SetFromNodeLabels(name='c1', nodeLabels=((instanceName,c1),))
			a.SetFromNodeLabels(name='c2', nodeLabels=((instanceName,c2),))
			a.SetFromNodeLabels(name='c3', nodeLabels=((instanceName,c3),))
			a.SetFromNodeLabels(name='c4', nodeLabels=((instanceName,c4),))
			a.SetFromNodeLabels(name='xfront', nodeLabels=((instanceName,xfront),))
			a.SetFromNodeLabels(name='xback', nodeLabels=((instanceName,xback),))
			a.SetFromNodeLabels(name='yfront', nodeLabels=((instanceName,yfront),))
			a.SetFromNodeLabels(name='yback', nodeLabels=((instanceName,yback),))


			if not error:


					for i,k in zip(yfront,yback):
						a.SetFromNodeLabels(name='yfront%s' % (i), nodeLabels=((instanceName,[i]),))
						a.SetFromNodeLabels(name='yback%s' % (k), nodeLabels=((instanceName,[k]),))

					for i,k in zip(xfront,xback):
						a.SetFromNodeLabels(name='xfront%s' % (i), nodeLabels=((instanceName,[i]),))
						a.SetFromNodeLabels(name='xback%s' % (k), nodeLabels=((instanceName,[k]),))

					## Constraints ##


					for i in mdb.models[modelName].constraints.keys():
							del mdb.models[modelName].constraints[i]

					if x2d_axis and y2d_axis:
						if y2d_axis:
							for i,k in zip(yfront,yback):
								mdb.models[modelName].Equation(name='1-TB%s' % i, terms=((1.0, 'yfront%s' % i, 1), (-1.0, 'yback%s' % k, 1), (-1.0, 'DN1', 1)))
							for i,k in zip(yfront,yback):
								mdb.models[modelName].Equation(name='2-TB%s' % i, terms=((1.0, 'yfront%s' % i, 2), (-1.0, 'yback%s' % k, 2), (-1.0, 'DN3', 2)))

						if x2d_axis:
							for i,k in zip(xfront,xback):
								mdb.models[modelName].Equation(name='1-FB%s' % i, terms=((1.0, 'xfront%s' % i, 1), (-1.0, 'xback%s' % k, 1), (-1.0, 'DN2', 1)))
							for i,k in zip(xfront,xback):
								mdb.models[modelName].Equation(name='2-FB%s' % i, terms=((1.0, 'xfront%s' % i, 2), (-1.0, 'xback%s' % k, 2), (-1.0, 'DN1', 2)))


						mdb.models[modelName].Equation(name='1-c42', terms=((1.0, 'c4', 1), (-1.0, 'c2', 1), (1.0, 'DN1', 1)))
						mdb.models[modelName].Equation(name='1-c21', terms=((1.0, 'c2', 1), (-1.0, 'c1', 1), (1.0, 'DN2', 1)))
						mdb.models[modelName].Equation(name='1-c13', terms=((1.0, 'c1', 1), (-1.0, 'c3', 1), (-1.0, 'DN1', 1)))


						mdb.models[modelName].Equation(name='2-c42', terms=((1.0, 'c4', 2), (-1.0, 'c2', 2), (1.0, 'DN3', 2)))
						mdb.models[modelName].Equation(name='2-c21', terms=((1.0, 'c2', 2), (-1.0, 'c1', 2), (1.0, 'DN1', 2)))
						mdb.models[modelName].Equation(name='2-c13', terms=((1.0, 'c1', 2), (-1.0, 'c3', 2), (-1.0, 'DN3', 2)))

						region = a.sets['DN1']
						mdb.models[modelName].DisplacementBC(name='BCXY', createStepName='Step-1',
							region=region, u1=float(x2d_displacement), u2=float(y2d_displacement), u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET,
							amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='',
							localCsys=None)


					else:
						if y2d_axis:
							for i,k in zip(yfront,yback):
								mdb.models[modelName].Equation(name='1-TB%s' % i, terms=((1.0, 'yfront%s' % i, 1), (-1.0, 'yback%s' % k, 1)))
							for i,k in zip(yfront,yback):
								mdb.models[modelName].Equation(name='2-TB%s' % i, terms=((1.0, 'yfront%s' % i, 2), (-1.0, 'yback%s' % k, 2), (-1.0, 'DN1', 2)))
							mdb.models[modelName].Equation(name='1-c42', terms=((1.0, 'c4', 1), (-1.0, 'c2', 1)))
							mdb.models[modelName].Equation(name='1-c13', terms=((1.0, 'c1', 1), (-1.0, 'c3', 1)))
							mdb.models[modelName].Equation(name='2-c42', terms=((1.0, 'c4', 2), (-1.0, 'c2', 2), (1.0, 'DN1', 2)))
							mdb.models[modelName].Equation(name='2-c13', terms=((1.0, 'c1', 2), (-1.0, 'c3', 2), (-1.0, 'DN1', 2)))
							
							if x2d_displacement or y2d_displacement:
								print("Displacement can only be applied to full pbc models")
								error3 = 0
								error = True
								message3 = ctypes.windll.user32.MessageBoxA
								message3(error3, 'Displacement can only be applied to full pbc models','Error', 0x30 | 0x0)
								continue
								
						if x2d_axis:
							for i,k in zip(xfront,xback):
								mdb.models[modelName].Equation(name='1-FB%s' % i, terms=((1.0, 'xfront%s' % i, 1), (-1.0, 'xback%s' % k, 1), (-1.0, 'DN2', 1)))
							for i,k in zip(xfront,xback):
								mdb.models[modelName].Equation(name='2-FB%s' % i, terms=((1.0, 'xfront%s' % i, 2), (-1.0, 'xback%s' % k, 2), (-1.0, 'DN2', 2)))
							mdb.models[modelName].Equation(name='1-c43', terms=((1.0, 'c4', 1), (-1.0, 'c3', 1), (1.0, 'DN2', 1)))
							mdb.models[modelName].Equation(name='1-c21', terms=((1.0, 'c2', 1), (-1.0, 'c1', 1), (1.0, 'DN2', 1)))
							mdb.models[modelName].Equation(name='2-c43', terms=((1.0, 'c4', 2), (-1.0, 'c3', 2), (1.0, 'DN2', 2)))
							mdb.models[modelName].Equation(name='2-c21', terms=((1.0, 'c2', 2), (-1.0, 'c1', 2), (1.0, 'DN2', 2)))

							if x2d_displacement or y2d_displacement:
								print("Displacement can only be applied to full pbc models")
								error3 = 0
								error = True
								message3 = ctypes.windll.user32.MessageBoxA
								message3(error3, 'Displacement can only be applied to full pbc models','Error', 0x30 | 0x0)
								continue
					for i in session.xyDataObjects.keys():
						del session.xyDataObjects['%s' % (i)]

		else:
			for i in a.features.keys():
				if i.startswith('DN'):
					del a.features['%s' % (i)]
			a.ReferencePoint(point=(Max + 1 * abs(Max - Mnx), May - 0.5 * (May - Mny), Maz - 0.5 * (Maz - Mnz)))
			a.ReferencePoint(point=(Max + 0.8 * abs(Max - Mnx), May - 0.5 * (May - Mny), Maz - 0.5 * (Maz - Mnz)))
			a.ReferencePoint(point=(Max + 0.6 * abs(Max - Mnx), May - 0.5 * (May - Mny), Maz - 0.5 * (Maz - Mnz)))
			a.ReferencePoint(point=(Max + 0.4 * abs(Max - Mnx), May - 0.5 * (May - Mny), Maz - 0.5 * (Maz - Mnz)))
			a.ReferencePoint(point=(Max + 0.2 * abs(Max - Mnx), May - 0.5 * (May - Mny), Maz - 0.5 * (Maz - Mnz)))
			a.ReferencePoint(point=(Max, May - 0.5 * (May - Mny), Maz - 0.5 * (Maz - Mnz)))

			r1 = a.referencePoints
			c1 = []
			c2 = []
			c3 = []
			c4 = []
			c5 = []
			c6 = []
			c7 = []
			c8 = []
			zfront = []
			zback = []
			xfront = []
			xback = []
			yfront = []
			yback = []
			xfrontbc = []
			xbackbc = []
			zfrontbc = []
			zbackbc = []
			yfrontbc = []
			ybackbc = []
			xy1edgexyz = {}
			xy2edgexyz = {}
			xy4edgexyz = {}
			xy3edgexyz = {}
			xz1edgexyz = {}
			xz2edgexyz = {}
			xz4edgexyz = {}
			xz3edgexyz = {}
			yz1edgexyz = {}
			yz4edgexyz = {}
			yz2edgexyz = {}
			yz3edgexyz = {}
			xy1edge = []
			xy2edge = []
			xy4edge = []
			xy3edge = []
			xz1edge = []
			xz4edge = []
			xz2edge = []
			xz3edge = []
			yz1edge = []
			yz2edge = []
			yz4edge = []
			yz3edge = []
			xfrontxyz = {}
			xbackxyz = {}
			yfrontxyz = {}
			ybackxyz = {}
			zfrontxyz = {}
			zbackxyz = {}
			xfrontbcxyz = {}
			xbackbcxyz = {}
			yfrontbcxyz = {}
			ybackbcxyz = {}
			zfrontbcxyz = {}
			zbackbcxyz = {}
			d = 6
			for i in r1.keys():
				refPoints1 = (r1[i],)
				a.Set(referencePoints=refPoints1, name='DN%s' % (d))
				d = d - 1

				#Boundary nodes#

				for i in Nodeset:
					if (Mnx + mesherror) < i.coordinates[0] < (Max - mesherror) and (Mny + mesherror) < i.coordinates[
						1] < (May - mesherror) and (Mnz + mesherror) < i.coordinates[2] < (Maz - mesherror):
						continue
					if abs(i.coordinates[0] - Max) <= mesherror:
						xfrontbcxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Mnx) <= mesherror:
						xbackbcxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[2] - Maz) <= mesherror:
						zfrontbcxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[2] - Mnz) <= mesherror:
						zbackbcxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[1] - May) <= mesherror:
						yfrontbcxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[1] - Mny) <= mesherror:
						ybackbcxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[1] - May) <= mesherror and abs(
							i.coordinates[2] - Maz) <= mesherror:
						c1.insert(0, i.label)
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[1] - May) <= mesherror and abs(
							i.coordinates[2] - Maz) <= mesherror:
						c2.insert(0, i.label)
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[1] - May) <= mesherror and abs(
							i.coordinates[2] - Mnz) <= mesherror:
						c3.insert(0, i.label)
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[1] - May) <= mesherror and abs(
							i.coordinates[2] - Mnz) <= mesherror:
						c4.insert(0, i.label)
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[1] - Mny) <= mesherror and abs(
							i.coordinates[2] - Maz) <= mesherror:
						c5.insert(0, i.label)
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[1] - Mny) <= mesherror and abs(
							i.coordinates[2] - Maz) <= mesherror:
						c6.insert(0, i.label)
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[1] - Mny) <= mesherror and abs(
							i.coordinates[2] - Mnz) <= mesherror:
						c7.insert(0, i.label)
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[1] - Mny) <= mesherror and abs(
							i.coordinates[2] - Mnz) <= mesherror:
						c8.insert(0, i.label)
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[1] - May) <= mesherror and abs(
							i.coordinates[2] - Maz) > mesherror and abs(i.coordinates[2] - Mnz) > mesherror:
						xy1edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[1] - Mny) <= mesherror and abs(
							i.coordinates[2] - Maz) > mesherror and abs(i.coordinates[2] - Mnz) > mesherror:
						xy4edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[1] - May) <= mesherror and abs(
							i.coordinates[2] - Maz) > mesherror and abs(i.coordinates[2] - Mnz) > mesherror:
						xy2edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[1] - Mny) <= mesherror and abs(
							i.coordinates[2] - Maz) > mesherror and abs(i.coordinates[2] - Mnz) > mesherror:
						xy3edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[2] - Maz) <= mesherror and abs(
							i.coordinates[1] - May) > mesherror and abs(i.coordinates[1] - Mny) > mesherror:
						xz1edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[2] - Mnz) <= mesherror and abs(
							i.coordinates[1] - May) > mesherror and abs(i.coordinates[1] - Mny) > mesherror:
						xz4edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[2] - Maz) <= mesherror and abs(
							i.coordinates[1] - May) > mesherror and abs(i.coordinates[1] - Mny) > mesherror:
						xz2edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[2] - Mnz) <= mesherror and abs(
							i.coordinates[1] - May) > mesherror and abs(i.coordinates[1] - Mny) > mesherror:
						xz3edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[2] - Maz) <= mesherror and abs(i.coordinates[1] - May) <= mesherror and abs(
							i.coordinates[0] - Max) > mesherror and abs(i.coordinates[0] - Mnx) > mesherror:
						yz1edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[2] - Maz) <= mesherror and abs(i.coordinates[1] - Mny) <= mesherror and abs(
							i.coordinates[0] - Max) > mesherror and abs(i.coordinates[0] - Mnx) > mesherror:
						yz2edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[2] - Mnz) <= mesherror and abs(i.coordinates[1] - May) <= mesherror and abs(
							i.coordinates[0] - Max) > mesherror and abs(i.coordinates[0] - Mnx) > mesherror:
						yz4edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[2] - Mnz) <= mesherror and abs(i.coordinates[1] - Mny) <= mesherror and abs(
							i.coordinates[0] - Max) > mesherror and abs(i.coordinates[0] - Mnx) > mesherror:
						yz3edgexyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Max) <= mesherror and abs(i.coordinates[1] - May) > mesherror and abs(
							i.coordinates[1] - Mny) > mesherror and abs(i.coordinates[2] - Maz) > mesherror and abs(
						i.coordinates[2] - Mnz) > mesherror:
						xfrontxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[0] - Mnx) <= mesherror and abs(i.coordinates[1] - May) > mesherror and abs(
							i.coordinates[1] - Mny) > mesherror and abs(i.coordinates[2] - Maz) > mesherror and abs(
						i.coordinates[2] - Mnz) > mesherror:
						xbackxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[2] - Maz) <= mesherror and abs(i.coordinates[1] - May) > mesherror and abs(
							i.coordinates[1] - Mny) > mesherror and abs(i.coordinates[0] - Max) > mesherror and abs(
						i.coordinates[0] - Mnx) > mesherror:
						zfrontxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[2] - Mnz) <= mesherror and abs(i.coordinates[1] - May) > mesherror and abs(
							i.coordinates[1] - Mny) > mesherror and abs(i.coordinates[0] - Max) > mesherror and abs(
						i.coordinates[0] - Mnx) > mesherror:
						zbackxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[1] - May) <= mesherror and abs(i.coordinates[0] - Max) > mesherror and abs(
							i.coordinates[0] - Mnx) > mesherror and abs(i.coordinates[2] - Maz) > mesherror and abs(
						i.coordinates[2] - Mnz) > mesherror:
						yfrontxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]
					if abs(i.coordinates[1] - Mny) <= mesherror and abs(i.coordinates[0] - Max) > mesherror and abs(
							i.coordinates[0] - Mnx) > mesherror and abs(i.coordinates[2] - Maz) > mesherror and abs(
						i.coordinates[2] - Mnz) > mesherror:
						ybackxyz[i.label] = [i.coordinates[0], i.coordinates[1], i.coordinates[2]]

			for i in xfrontxyz.keys():
				for k in xbackxyz.keys():
					if abs(xfrontxyz[i][1] - xbackxyz[k][1]) <= mesherror and abs(xfrontxyz[i][2] - xbackxyz[k][2]) <= mesherror:
						xfront.append(i)
						xback.append(k)

			for i in yfrontxyz.keys():
				for k in ybackxyz.keys():
					if abs(yfrontxyz[i][0] - ybackxyz[k][0]) <= mesherror and abs(yfrontxyz[i][2] - ybackxyz[k][2]) <= mesherror:
						yfront.append(i)
						yback.append(k)

			for i in zfrontxyz.keys():
				for k in zbackxyz.keys():
					if abs(zfrontxyz[i][0] - zbackxyz[k][0]) <= mesherror and abs(zfrontxyz[i][1] - zbackxyz[k][1]) <= mesherror:
						zfront.append(i)
						zback.append(k)

			for i in xfrontbcxyz.keys():
				for k in xbackbcxyz.keys():
					if abs(xfrontbcxyz[i][1] - xbackbcxyz[k][1]) <= mesherror and abs(xfrontbcxyz[i][2] - xbackbcxyz[k][2]) <= mesherror:
						xfrontbc.append(i)
						xbackbc.append(k)

			for i in yfrontbcxyz.keys():
				for k in ybackbcxyz.keys():
					if abs(yfrontbcxyz[i][0] - ybackbcxyz[k][0]) <= mesherror and abs(yfrontbcxyz[i][2] - ybackbcxyz[k][2]) <= mesherror:
						yfrontbc.append(i)
						ybackbc.append(k)

			for i in zfrontbcxyz.keys():
				for k in zbackbcxyz.keys():
					if abs(zfrontbcxyz[i][0] - zbackbcxyz[k][0]) <= mesherror and abs(zfrontbcxyz[i][1] - zbackbcxyz[k][1]) <= mesherror:
						zfrontbc.append(i)
						zbackbc.append(k)

			for i in xy1edgexyz.keys():
				for k in xy2edgexyz.keys():
					if abs(xy1edgexyz[i][1] - xy2edgexyz[k][1]) <= mesherror and abs(xy1edgexyz[i][2] - xy2edgexyz[k][2]) <= mesherror:
						xy1edge.append(i)
						xy2edge.append(k)

			for i in xy2edge:
				for k in xy3edgexyz.keys():
					if abs(xy2edgexyz[i][0] - xy3edgexyz[k][0]) <= mesherror and abs(xy2edgexyz[i][2] - xy3edgexyz[k][2]) <= mesherror:
						xy3edge.append(k)
			for i in xy3edge:
				for k in xy4edgexyz.keys():
					if abs(xy3edgexyz[i][1] - xy4edgexyz[k][1]) <= mesherror and abs(xy3edgexyz[i][2] - xy4edgexyz[k][2]) <= mesherror:
						xy4edge.append(k)

			for i in yz1edgexyz.keys():
				for k in yz4edgexyz.keys():
					if abs(yz1edgexyz[i][0] - yz4edgexyz[k][0]) <= mesherror and abs(yz1edgexyz[i][1] - yz4edgexyz[k][1]) <= mesherror:
						yz1edge.append(i)
						yz4edge.append(k)
			for i in yz4edge:
				for k in yz3edgexyz.keys():
					if abs(yz4edgexyz[i][0] - yz3edgexyz[k][0]) <= mesherror and abs(yz4edgexyz[i][2] - yz3edgexyz[k][2]) <= mesherror:
						yz3edge.append(k)
			for i in yz3edge:
				for k in yz2edgexyz.keys():
					if abs(yz3edgexyz[i][0] - yz2edgexyz[k][0]) <= mesherror and abs(yz3edgexyz[i][1] - yz2edgexyz[k][1]) <= mesherror:
						yz2edge.append(k)

			for i in xz1edgexyz.keys():
				for k in xz2edgexyz.keys():
					if abs(xz1edgexyz[i][1] - xz2edgexyz[k][1]) <= mesherror and abs(xz1edgexyz[i][2] - xz2edgexyz[k][2]) <= mesherror:
						xz1edge.append(i)
						xz2edge.append(k)
			for i in xz2edge:
				for k in xz3edgexyz.keys():
					if abs(xz2edgexyz[i][0] - xz3edgexyz[k][0]) <= mesherror and abs(xz2edgexyz[i][1] - xz3edgexyz[k][1]) <= mesherror:
						xz3edge.append(k)
			for i in xz3edge:
				for k in xz4edgexyz.keys():
					if abs(xz3edgexyz[i][1] - xz4edgexyz[k][1]) <= mesherror and abs(xz3edgexyz[i][2] - xz4edgexyz[k][2]) <= mesherror:
						xz4edge.append(k)

			#Sets#

			a.SetFromNodeLabels(name='c1', nodeLabels=((instanceName, c1),))
			a.SetFromNodeLabels(name='c2', nodeLabels=((instanceName, c2),))
			a.SetFromNodeLabels(name='c3', nodeLabels=((instanceName, c3),))
			a.SetFromNodeLabels(name='c4', nodeLabels=((instanceName, c4),))
			a.SetFromNodeLabels(name='c5', nodeLabels=((instanceName, c5),))
			a.SetFromNodeLabels(name='c6', nodeLabels=((instanceName, c6),))
			a.SetFromNodeLabels(name='c7', nodeLabels=((instanceName, c7),))
			a.SetFromNodeLabels(name='c8', nodeLabels=((instanceName, c8),))
			a.SetFromNodeLabels(name='xy1edge', nodeLabels=((instanceName, xy1edge),))
			a.SetFromNodeLabels(name='xy4edge', nodeLabels=((instanceName, xy4edge),))
			a.SetFromNodeLabels(name='xy2edge', nodeLabels=((instanceName, xy2edge),))
			a.SetFromNodeLabels(name='xy3edge', nodeLabels=((instanceName, xy3edge),))
			a.SetFromNodeLabels(name='xz1edge', nodeLabels=((instanceName, xz1edge),))
			a.SetFromNodeLabels(name='xz4edge', nodeLabels=((instanceName, xz4edge),))
			a.SetFromNodeLabels(name='xz2edge', nodeLabels=((instanceName, xz2edge),))
			a.SetFromNodeLabels(name='xz3edge', nodeLabels=((instanceName, xz3edge),))
			a.SetFromNodeLabels(name='yz1edge', nodeLabels=((instanceName, yz1edge),))
			a.SetFromNodeLabels(name='yz2edge', nodeLabels=((instanceName, yz2edge),))
			a.SetFromNodeLabels(name='yz4edge', nodeLabels=((instanceName, yz4edge),))
			a.SetFromNodeLabels(name='yz3edge', nodeLabels=((instanceName, yz3edge),))
			a.SetFromNodeLabels(name='xfront', nodeLabels=((instanceName, xfront),))
			a.SetFromNodeLabels(name='xback', nodeLabels=((instanceName, xback),))
			a.SetFromNodeLabels(name='zfront', nodeLabels=((instanceName, zfront),))
			a.SetFromNodeLabels(name='zback', nodeLabels=((instanceName, zback),))
			a.SetFromNodeLabels(name='yfront', nodeLabels=((instanceName, yfront),))
			a.SetFromNodeLabels(name='yback', nodeLabels=((instanceName, yback),))

			a = mdb.models[modelName].rootAssembly
			Nodeset = mdb.models[modelName].rootAssembly.instances[instanceName].nodes
			mdb.models[modelName].StaticStep(name='Step-1', previous='Initial')




			if error == False:

				for i, k in zip(yfront, yback):
					a.SetFromNodeLabels(name='yfront%s' % (i), nodeLabels=((instanceName, [i]),))
					a.SetFromNodeLabels(name='yback%s' % (k), nodeLabels=((instanceName, [k]),))

				for i, k in zip(xfront, xback):
					a.SetFromNodeLabels(name='xfront%s' % (i), nodeLabels=((instanceName, [i]),))
					a.SetFromNodeLabels(name='xback%s' % (k), nodeLabels=((instanceName, [k]),))

				for i, k in zip(zfront, zback):
					a.SetFromNodeLabels(name='zfront%s' % (i), nodeLabels=((instanceName, [i]),))
					a.SetFromNodeLabels(name='zback%s' % (k), nodeLabels=((instanceName, [k]),))

				for i, k, j, l in zip(xy1edge, xy2edge, xy3edge, xy4edge):
					a.SetFromNodeLabels(name='xy1edge%s' % (i), nodeLabels=((instanceName, [i]),))
					a.SetFromNodeLabels(name='xy2edge%s' % (k), nodeLabels=((instanceName, [k]),))
					a.SetFromNodeLabels(name='xy3edge%s' % (j), nodeLabels=((instanceName, [j]),))
					a.SetFromNodeLabels(name='xy4edge%s' % (l), nodeLabels=((instanceName, [l]),))

				for i, k, j, l in zip(xz1edge, xz2edge, xz3edge, xz4edge):
					a.SetFromNodeLabels(name='xz1edge%s' % (i), nodeLabels=((instanceName, [i]),))
					a.SetFromNodeLabels(name='xz2edge%s' % (k), nodeLabels=((instanceName, [k]),))
					a.SetFromNodeLabels(name='xz3edge%s' % (j), nodeLabels=((instanceName, [j]),))
					a.SetFromNodeLabels(name='xz4edge%s' % (l), nodeLabels=((instanceName, [l]),))

				for i, k, j, l in zip(yz1edge, yz2edge, yz3edge, yz4edge):
					a.SetFromNodeLabels(name='yz1edge%s' % (i), nodeLabels=((instanceName, [i]),))
					a.SetFromNodeLabels(name='yz2edge%s' % (k), nodeLabels=((instanceName, [k]),))
					a.SetFromNodeLabels(name='yz3edge%s' % (j), nodeLabels=((instanceName, [j]),))
					a.SetFromNodeLabels(name='yz4edge%s' % (l), nodeLabels=((instanceName, [l]),))


			# Constrains#
				for i in mdb.models[modelName].constraints.keys():
					del mdb.models[modelName].constraints[i]
				if (x3d_axis and y3d_axis) or (x3d_axis and z3d_axis) or (y3d_axis and z3d_axis):
					if y3d_axis:
						for i,k in zip(yfront,yback):
							mdb.models[modelName].Equation(name='1-yfront-yback%s'%i, terms=((1.0, 'yfront%s'%i, 1), (-1.0, 'yback%s'%k, 1),(-1.0, 'DN4', 1)))
						for i,k in zip(yfront,yback):
							mdb.models[modelName].Equation(name='2-yfront-yback%s'%i, terms=((1.0, 'yfront%s'%i, 2), (-1.0, 'yback%s'%k, 2),(-1.0, 'DN1', 2)))
						for i,k in zip(yfront,yback):
							mdb.models[modelName].Equation(name='3-yfront-yback%s'%i, terms=((1.0, 'yfront%s'%i, 3), (-1.0, 'yback%s'%k, 3),(-1.0, 'DN6', 3)))

					if z3d_axis:
						for i,k in zip(zfront,zback):
							mdb.models[modelName].Equation(name='1-zfront-zback%s'%i, terms=((1.0, 'zfront%s'%i, 1), (-1.0, 'zback%s'%k, 1),(-1.0, 'DN5', 1)))
						for i,k in zip(zfront,zback):
							mdb.models[modelName].Equation(name='2-zfront-zback%s'%i, terms=((1.0, 'zfront%s'%i, 2), (-1.0, 'zback%s'%k, 2),(-1.0, 'DN6', 2)))
						for i,k in zip(zfront,zback):
							mdb.models[modelName].Equation(name='3-zfront-zback%s'%i, terms=((1.0, 'zfront%s'%i, 3), (-1.0, 'zback%s'%k, 3),(-1.0, 'DN2', 3)))

					if x3d_axis:
						for i,k in zip(xfront,xback):
							mdb.models[modelName].Equation(name='1-xfront-xback%s'%i, terms=((1.0, 'xfront%s'%i, 1), (-1.0, 'xback%s'%k, 1),(-1.0, 'DN3', 1)))
						for i,k in zip(xfront,xback):
							mdb.models[modelName].Equation(name='2-xfront-xback%s'%i, terms=((1.0, 'xfront%s'%i, 2), (-1.0, 'xback%s'%k, 2),(-1.0, 'DN4', 2)))
						for i,k in zip(xfront,xback):
							mdb.models[modelName].Equation(name='3-xfront-xback%s'%i, terms=((1.0, 'xfront%s'%i, 3), (-1.0, 'xback%s'%k, 3),(-1.0, 'DN5', 3)))


					mdb.models[modelName].Equation(name='1-c12', terms=((1.0, 'c6', 1), (-1.0, 'c2', 1),(1.0, 'DN4', 1)))
					mdb.models[modelName].Equation(name='1-c23', terms=((1.0, 'c2', 1), (-1.0, 'c3', 1),(-1.0, 'DN5', 1)))
					mdb.models[modelName].Equation(name='1-c34', terms=((1.0, 'c3', 1), (-1.0, 'c4', 1),(1.0, 'DN3', 1)))
					mdb.models[modelName].Equation(name='1-c45', terms=((1.0, 'c4', 1), (-1.0, 'c8', 1),(-1.0, 'DN4', 1)))
					mdb.models[modelName].Equation(name='1-c56', terms=((1.0, 'c8', 1), (-1.0, 'c5', 1),(1.0, 'DN5', 1)))
					mdb.models[modelName].Equation(name='1-c67', terms=((1.0, 'c5', 1), (-1.0, 'c1', 1),(1.0, 'DN4', 1)))
					mdb.models[modelName].Equation(name='1-c78', terms=((1.0, 'c1', 1), (-1.0, 'c7', 1),(-1.0, 'DN3', 1),(-1.0, 'DN4', 1),(-1.0, 'DN5', 1)))


					mdb.models[modelName].Equation(name='2-c12', terms=((1.0, 'c6', 2), (-1.0, 'c2', 2),(1.0, 'DN1', 2)))
					mdb.models[modelName].Equation(name='2-c23', terms=((1.0, 'c2', 2), (-1.0, 'c3', 2),(-1.0, 'DN6', 2)))
					mdb.models[modelName].Equation(name='2-c34', terms=((1.0, 'c3', 2), (-1.0, 'c4', 2),(1.0, 'DN4', 2)))
					mdb.models[modelName].Equation(name='2-c45', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2),(-1.0, 'DN1', 2)))
					mdb.models[modelName].Equation(name='2-c56', terms=((1.0, 'c8', 2), (-1.0, 'c5', 2),(1.0, 'DN6', 2)))
					mdb.models[modelName].Equation(name='2-c67', terms=((1.0, 'c5', 2), (-1.0, 'c1', 2),(1.0, 'DN1', 2)))
					mdb.models[modelName].Equation(name='2-c78', terms=((1.0, 'c1', 2), (-1.0, 'c7', 2),(-1.0, 'DN1', 2),(-1.0, 'DN4', 2),(-1.0, 'DN6', 2)))


					mdb.models[modelName].Equation(name='3-c12', terms=((1.0, 'c6', 3), (-1.0, 'c2', 3),(1.0, 'DN6', 3)))
					mdb.models[modelName].Equation(name='3-c23', terms=((1.0, 'c2', 3), (-1.0, 'c3', 3),(-1.0, 'DN2', 3)))
					mdb.models[modelName].Equation(name='3-c34', terms=((1.0, 'c3', 3), (-1.0, 'c4', 3),(1.0, 'DN5', 3)))
					mdb.models[modelName].Equation(name='3-c45', terms=((1.0, 'c4', 3), (-1.0, 'c8', 3),(-1.0, 'DN6', 3)))
					mdb.models[modelName].Equation(name='3-c56', terms=((1.0, 'c8', 3), (-1.0, 'c5', 3),(1.0, 'DN2', 3)))
					mdb.models[modelName].Equation(name='3-c67', terms=((1.0, 'c5', 3), (-1.0, 'c1', 3),(1.0, 'DN6', 3)))
					mdb.models[modelName].Equation(name='3-c78', terms=((1.0, 'c1', 3), (-1.0, 'c7', 3),(-1.0, 'DN2', 3),(-1.0, 'DN5', 3),(-1.0, 'DN6', 3)))


					for i,k,j,l in zip(xy1edge,xy2edge,xy3edge,xy4edge):
						mdb.models[modelName].Equation(name='1-xy1edge-xy2edge%s'%i, terms=((1.0, 'xy1edge%s'%i, 1), (-1.0, 'xy2edge%s'%k, 1),(-1.0, 'DN3', 1)))
						mdb.models[modelName].Equation(name='1-xy2edge-xy3edge%s'%k, terms=((1.0, 'xy2edge%s'%k, 1), (-1.0, 'xy3edge%s'%j, 1),(-1.0, 'DN4', 1)))
						mdb.models[modelName].Equation(name='1-xy3edge-xy4edge%s'%j, terms=((1.0, 'xy3edge%s'%j, 1), (-1.0, 'xy4edge%s'%l, 1),(1.0, 'DN3', 1)))
					for i,k,j,l in zip(xy1edge,xy2edge,xy3edge,xy4edge):
						mdb.models[modelName].Equation(name='2-xy1edge-xy2edge%s'%i, terms=((1.0, 'xy1edge%s'%i, 2), (-1.0, 'xy2edge%s'%k, 2),(-1.0, 'DN4', 2)))
						mdb.models[modelName].Equation(name='2-xy2edge-xy3edge%s'%k, terms=((1.0, 'xy2edge%s'%k, 2), (-1.0, 'xy3edge%s'%j, 2),(-1.0, 'DN1', 2)))
						mdb.models[modelName].Equation(name='2-xy3edge-xy4edge%s'%j, terms=((1.0, 'xy3edge%s'%j, 2), (-1.0, 'xy4edge%s'%l, 2),(1.0, 'DN4', 2)))
					for i,k,j,l in zip(xy1edge,xy2edge,xy3edge,xy4edge):
						mdb.models[modelName].Equation(name='3-xy1edge-xy2edge%s'%i, terms=((1.0, 'xy1edge%s'%i, 3), (-1.0, 'xy2edge%s'%k, 3),(-1.0, 'DN5', 3)))
						mdb.models[modelName].Equation(name='3-xy2edge-xy3edge%s'%k, terms=((1.0, 'xy2edge%s'%k, 3), (-1.0, 'xy3edge%s'%j, 3),(-1.0, 'DN6', 3)))
						mdb.models[modelName].Equation(name='3-xy3edge-xy4edge%s'%j, terms=((1.0, 'xy3edge%s'%j, 3), (-1.0, 'xy4edge%s'%l, 3),(1.0, 'DN5', 3)))


					for i,k,j,l in zip(xz1edge,xz2edge,xz3edge,xz4edge):
						mdb.models[modelName].Equation(name='1-xz1edge-xz2edge%s'%i, terms=((1.0, 'xz1edge%s'%i, 1), (-1.0, 'xz2edge%s'%k, 1),(-1.0, 'DN3', 1)))
						mdb.models[modelName].Equation(name='1-xz2edge-xz3edge%s'%k, terms=((1.0, 'xz2edge%s'%k, 1), (-1.0, 'xz3edge%s'%j, 1),(-1.0, 'DN5', 1)))
						mdb.models[modelName].Equation(name='1-xz3edge-xz4edge%s'%j, terms=((1.0, 'xz3edge%s'%j, 1), (-1.0, 'xz4edge%s'%l, 1),(1.0, 'DN3', 1)))
					for i,k,j,l in zip(xz1edge,xz2edge,xz3edge,xz4edge):
						mdb.models[modelName].Equation(name='2-xz1edge-xz2edge%s'%i, terms=((1.0, 'xz1edge%s'%i, 2), (-1.0, 'xz2edge%s'%k, 2),(-1.0, 'DN4', 2)))
						mdb.models[modelName].Equation(name='2-xz2edge-xz3edge%s'%k, terms=((1.0, 'xz2edge%s'%k, 2), (-1.0, 'xz3edge%s'%j, 2),(-1.0, 'DN6', 2)))
						mdb.models[modelName].Equation(name='2-xz3edge-xz4edge%s'%j, terms=((1.0, 'xz3edge%s'%j, 2), (-1.0, 'xz4edge%s'%l, 2),(1.0, 'DN4', 2)))
					for i,k,j,l in zip(xz1edge,xz2edge,xz3edge,xz4edge):
						mdb.models[modelName].Equation(name='3-xz1edge-xz2edge%s'%i, terms=((1.0, 'xz1edge%s'%i, 3), (-1.0, 'xz2edge%s'%k, 3),(-1.0, 'DN5', 3)))
						mdb.models[modelName].Equation(name='3-xz2edge-xz3edge%s'%k, terms=((1.0, 'xz2edge%s'%k, 3), (-1.0, 'xz3edge%s'%j, 3),(-1.0, 'DN2', 3)))
						mdb.models[modelName].Equation(name='3-xz3edge-xz4edge%s'%j, terms=((1.0, 'xz3edge%s'%j, 3), (-1.0, 'xz4edge%s'%l, 3),(1.0, 'DN5', 3)))


					for i,k,j,l in zip(yz1edge,yz2edge,yz3edge,yz4edge):
						mdb.models[modelName].Equation(name='1-yz1edge-yz2edge%s'%i, terms=((1.0, 'yz1edge%s'%i, 1), (-1.0, 'yz2edge%s'%k, 1),(-1.0, 'DN4', 1)))
						mdb.models[modelName].Equation(name='1-yz2edge-yz3edge%s'%k, terms=((1.0, 'yz2edge%s'%k, 1), (-1.0, 'yz3edge%s'%j, 1),(-1.0, 'DN5', 1)))
						mdb.models[modelName].Equation(name='1-yz3edge-rtbedge%s'%j, terms=((1.0, 'yz3edge%s'%j, 1), (-1.0, 'yz4edge%s'%l, 1),(1.0, 'DN4', 1)))
					for i,k,j,l in zip(yz1edge,yz2edge,yz3edge,yz4edge):
						mdb.models[modelName].Equation(name='2-yz1edge-yz2edge%s'%i, terms=((1.0, 'yz1edge%s'%i, 2), (-1.0, 'yz2edge%s'%k, 2),(-1.0, 'DN1', 2)))
						mdb.models[modelName].Equation(name='2-yz2edge-yz3edge%s'%k, terms=((1.0, 'yz2edge%s'%k, 2), (-1.0, 'yz3edge%s'%j, 2),(-1.0, 'DN6', 2)))
						mdb.models[modelName].Equation(name='2-yz3edge-rtbedge%s'%j, terms=((1.0, 'yz3edge%s'%j, 2), (-1.0, 'yz4edge%s'%l, 2),(1.0, 'DN1', 2)))
					for i,k,j,l in zip(yz1edge,yz2edge,yz3edge,yz4edge):
						mdb.models[modelName].Equation(name='3-yz1edge-yz2edge%s'%i, terms=((1.0, 'yz1edge%s'%i, 3), (-1.0, 'yz2edge%s'%k, 3),(-1.0, 'DN6', 3)))
						mdb.models[modelName].Equation(name='3-yz2edge-yz3edge%s'%k, terms=((1.0, 'yz2edge%s'%k, 3), (-1.0, 'yz3edge%s'%j, 3),(-1.0, 'DN2', 3)))
						mdb.models[modelName].Equation(name='3-yz3edge-rtbedge%s'%j, terms=((1.0, 'yz3edge%s'%j, 3), (-1.0, 'yz4edge%s'%l, 3),(1.0, 'DN6', 3)))
						
					region = a.sets['DN4']
					mdb.models[modelName].DisplacementBC(name='BCXY', createStepName='Step-1',
						region=region, u1=float(x3d_displacement), u2=float(y3d_displacement), u3=0, ur1=UNSET, ur2=UNSET, ur3=UNSET,
						amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='',
						localCsys=None)

					region = a.sets['DN5']
					mdb.models[modelName].DisplacementBC(name='BCXZ', createStepName='Step-1',
						region=region, u1=float(x3d_displacement), u2=0, u3=float(z3d_displacement), ur1=UNSET, ur2=UNSET, ur3=UNSET,
						amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='',
						localCsys=None)

					region = a.sets['DN6']
					mdb.models[modelName].DisplacementBC(name='BCYZ', createStepName='Step-1',
						region=region, u1=0, u2=float(y3d_displacement), u3=float(z3d_displacement), ur1=UNSET, ur2=UNSET, ur3=UNSET,
						amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='',
						localCsys=None)

				else:
					if y3d_axis:
						for i, k in zip(yfront, yback):
							mdb.models[modelName].Equation(name='1-yfront-yback%s' % i,
														   terms=((1.0, 'yfront%s' % i, 1), (-1.0, 'yback%s' % k, 1), (-1.0, 'DN4', 1)))
						for i, k in zip(yfront, yback):
							mdb.models[modelName].Equation(name='2-yfront-yback%s' % i,
														   terms=((1.0, 'yfront%s' % i, 2), (-1.0, 'yback%s' % k, 2), (-1.0, 'DN1', 2)))
						for i, k in zip(yfront, yback):
							mdb.models[modelName].Equation(name='3-yfront-yback%s' % i,
														   terms=((1.0, 'yfront%s' % i, 3), (-1.0, 'yback%s' % k, 3), (-1.0, 'DN6', 3)))

						mdb.models[modelName].Equation(name='1-c45', terms=((1.0, 'c4', 1), (-1.0, 'c8', 1), (-1.0, 'DN4', 1)))
						mdb.models[modelName].Equation(name='1-c67', terms=((1.0, 'c5', 1), (-1.0, 'c1', 1), (1.0, 'DN4', 1)))
						mdb.models[modelName].Equation(name='1-c12', terms=((1.0, 'c6', 1), (-1.0, 'c2', 1), (1.0, 'DN4', 1)))

						mdb.models[modelName].Equation(name='2-c12', terms=((1.0, 'c6', 2), (-1.0, 'c2', 2), (1.0, 'DN1', 2)))
						mdb.models[modelName].Equation(name='2-c45', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2), (-1.0, 'DN1', 2)))
						mdb.models[modelName].Equation(name='2-c67', terms=((1.0, 'c5', 2), (-1.0, 'c1', 2), (1.0, 'DN1', 2)))

						mdb.models[modelName].Equation(name='3-c12', terms=((1.0, 'c6', 3), (-1.0, 'c2', 3), (1.0, 'DN6', 3)))
						mdb.models[modelName].Equation(name='3-c45', terms=((1.0, 'c4', 3), (-1.0, 'c8', 3), (-1.0, 'DN6', 3)))
						mdb.models[modelName].Equation(name='3-c67', terms=((1.0, 'c5', 3), (-1.0, 'c1', 3), (1.0, 'DN6', 3)))

						for i, k, j, l in zip(xy1edge, xy2edge, xy3edge, xy4edge):
							mdb.models[modelName].Equation(name='1-xy2edge-xy3edge%s' % k,
														   terms=((1.0, 'xy2edge%s' % k, 1), (-1.0, 'xy3edge%s' % j, 1), (-1.0, 'DN4', 1)))
							mdb.models[modelName].Equation(name='1-xy1edge-xy4edge%s' % i,
														   terms=((1.0, 'xy1edge%s' % i, 1), (-1.0, 'xy4edge%s' % l, 1), (-1.0, 'DN4', 1)))
						for i, k, j, l in zip(xy1edge, xy2edge, xy3edge, xy4edge):
							mdb.models[modelName].Equation(name='2-xy2edge-xy3edge%s' % k,
														   terms=((1.0, 'xy2edge%s' % k, 2), (-1.0, 'xy3edge%s' % j, 2), (-1.0, 'DN1', 2)))
							mdb.models[modelName].Equation(name='2-xy1edge-xy4edge%s' % i,
														   terms=((1.0, 'xy1edge%s' % i, 2), (-1.0, 'xy4edge%s' % l, 2), (-1.0, 'DN1', 2)))
						for i, k, j, l in zip(xy1edge, xy2edge, xy3edge, xy4edge):
							mdb.models[modelName].Equation(name='3-xy2edge-xy3edge%s' % k,
														   terms=((1.0, 'xy2edge%s' % k, 3), (-1.0, 'xy3edge%s' % j, 3), (-1.0, 'DN6', 3)))
							mdb.models[modelName].Equation(name='3-xy1edge-xy4edge%s' % i,
														   terms=((1.0, 'xy1edge%s' % i, 3), (-1.0, 'xy4edge%s' % l, 3), (-1.0, 'DN6', 3)))
						for i, k, j, l in zip(yz1edge, yz2edge, yz3edge, yz4edge):
							mdb.models[modelName].Equation(name='1-yz1edge-yz2edge%s' % i,
														   terms=((1.0, 'yz1edge%s' % i, 1), (-1.0, 'yz2edge%s' % k, 1), (-1.0, 'DN4', 1)))
							mdb.models[modelName].Equation(name='1-yz4edge-yz3edge%s' % l,
														   terms=((1.0, 'yz4edge%s' % l, 1), (-1.0, 'yz3edge%s' % j, 1), (-1.0, 'DN4', 1)))
						for i, k, j, l in zip(yz1edge, yz2edge, yz3edge, yz4edge):
							mdb.models[modelName].Equation(name='2-yz1edge-yz2edge%s' % i,
														   terms=((1.0, 'yz1edge%s' % i, 2), (-1.0, 'yz2edge%s' % k, 2), (-1.0, 'DN1', 2)))
							mdb.models[modelName].Equation(name='2-yz4edge-yz3edge%s' % l,
														   terms=((1.0, 'yz4edge%s' % l, 2), (-1.0, 'yz3edge%s' % j, 2), (-1.0, 'DN1', 2)))
						for i, k, j, l in zip(yz1edge, yz2edge, yz3edge, yz4edge):
							mdb.models[modelName].Equation(name='3-yz1edge-yz2edge%s' % i,
														   terms=((1.0, 'yz1edge%s' % i, 3), (-1.0, 'yz2edge%s' % k, 3), (-1.0, 'DN6', 3)))
							mdb.models[modelName].Equation(name='3-yz4edge-yz3edge%s' % l,
														   terms=((1.0, 'yz4edge%s' % l, 3), (-1.0, 'yz3edge%s' % j, 3), (-1.0, 'DN6', 3)))
														   
						if x3d_displacement or x3d_displacement or z3d_displacement:
							print("Displacement can only be applied to full pbc models")
							error3 = 0
							error = True
							message3 = ctypes.windll.user32.MessageBoxA
							message3(error3, 'Displacement can only be applied to full pbc models','Error', 0x30 | 0x0)
							continue
							
					if z3d_axis:
						for i, k in zip(zfront, zback):
							mdb.models[modelName].Equation(name='1-zfront-zback%s' % i,
														   terms=((1.0, 'zfront%s' % i, 1), (-1.0, 'zback%s' % k, 1), (-1.0, 'DN5', 1)))
						for i, k in zip(zfront, zback):
							mdb.models[modelName].Equation(name='2-zfront-zback%s' % i,
														   terms=((1.0, 'zfront%s' % i, 2), (-1.0, 'zback%s' % k, 2), (-1.0, 'DN6', 2)))
						for i, k in zip(zfront, zback):
							mdb.models[modelName].Equation(name='3-zfront-zback%s' % i,
														   terms=((1.0, 'zfront%s' % i, 3), (-1.0, 'zback%s' % k, 3), (-1.0, 'DN2', 3)))

						mdb.models[modelName].Equation(name='1-c23', terms=((1.0, 'c2', 1), (-1.0, 'c3', 1), (-1.0, 'DN5', 1)))
						mdb.models[modelName].Equation(name='1-c56', terms=((1.0, 'c8', 1), (-1.0, 'c5', 1), (1.0, 'DN5', 1)))
						mdb.models[modelName].Equation(name='1-c14', terms=((1.0, 'c1', 1), (-1.0, 'c4', 1),(-1.0, 'DN5', 1)))

						mdb.models[modelName].Equation(name='2-c23', terms=((1.0, 'c2', 2), (-1.0, 'c3', 2), (-1.0, 'DN6', 2)))
						mdb.models[modelName].Equation(name='2-c56', terms=((1.0, 'c8', 2), (-1.0, 'c5', 2), (1.0, 'DN6', 2)))
						mdb.models[modelName].Equation(name='2-c14', terms=((1.0, 'c1', 2), (-1.0, 'c4', 2),(-1.0, 'DN6', 2)))

						mdb.models[modelName].Equation(name='3-c23', terms=((1.0, 'c2', 3), (-1.0, 'c3', 3), (-1.0, 'DN2', 3)))
						mdb.models[modelName].Equation(name='3-c56', terms=((1.0, 'c8', 3), (-1.0, 'c5', 3), (1.0, 'DN2', 3)))
						mdb.models[modelName].Equation(name='3-c14', terms=((1.0, 'c1', 3), (-1.0, 'c4', 3), (-1.0, 'DN2', 3)))

						for i, k, j, l in zip(xz1edge, xz2edge, xz3edge, xz4edge):
							mdb.models[modelName].Equation(name='1-xz1edge-xz4edge%s' % l,
														   terms=((1.0, 'xz1edge%s' % i, 1), (-1.0, 'xz4edge%s' % l, 1), (-1.0, 'DN5', 1)))
							mdb.models[modelName].Equation(name='1-xz2edge-xz3edge%s' % k,
														   terms=((1.0, 'xz2edge%s' % k, 1), (-1.0, 'xz3edge%s' % j, 1), (-1.0, 'DN5', 1)))

						for i, k, j, l in zip(xz1edge, xz2edge, xz3edge, xz4edge):
							mdb.models[modelName].Equation(name='2-xz2edge-xz3edge%s' % k,
														   terms=((1.0, 'xz2edge%s' % k, 2), (-1.0, 'xz3edge%s' % j, 2), (-1.0, 'DN6', 2)))
							mdb.models[modelName].Equation(name='2-xz1edge-xz4edge%s' % l,
														   terms=((1.0, 'xz1edge%s' % i, 2), (-1.0, 'xz4edge%s' % l, 2), (-1.0, 'DN6', 2)))

						for i, k, j, l in zip(xz1edge, xz2edge, xz3edge, xz4edge):
							mdb.models[modelName].Equation(name='3-xz2edge-xz3edge%s' % k,
														   terms=((1.0, 'xz2edge%s' % k, 3), (-1.0, 'xz3edge%s' % j, 3), (-1.0, 'DN2', 3)))
							mdb.models[modelName].Equation(name='3-xz1edge-xz4edge%s' % l,
														   terms=((1.0, 'xz1edge%s' % i, 3), (-1.0, 'xz4edge%s' % l, 3), (-1.0, 'DN2', 3)))

						for i, k, j, l in zip(yz1edge, yz2edge, yz3edge, yz4edge):
							mdb.models[modelName].Equation(name='1-yz2edge-yz3edge%s' % k,
															terms=((1.0, 'yz2edge%s' % k, 1), (-1.0, 'yz3edge%s' % j, 1), (-1.0, 'DN5', 1)))
							mdb.models[modelName].Equation(name='1-yz1edge-yz4edge%s' % l,
															terms=((1.0, 'yz1edge%s' % i, 1), (-1.0, 'yz4edge%s' % l, 1), (-1.0, 'DN5', 1)))

						for i, k, j, l in zip(yz1edge, yz2edge, yz3edge, yz4edge):

							mdb.models[modelName].Equation(name='2-yz2edge-yz3edge%s' % k,
														   terms=((1.0, 'yz2edge%s' % k, 2), (-1.0, 'yz3edge%s' % j, 2), (-1.0, 'DN6', 2)))
							mdb.models[modelName].Equation(name='2-yz1edge-yz4edge%s' % l,
														   terms=((1.0, 'yz1edge%s' % i, 2), (-1.0, 'yz4edge%s' % l, 2), (-1.0, 'DN6', 2)))

						for i, k, j, l in zip(yz1edge, yz2edge, yz3edge, yz4edge):
							mdb.models[modelName].Equation(name='3-yz2edge-yz3edge%s' % k,
														   terms=((1.0, 'yz2edge%s' % k, 3), (-1.0, 'yz3edge%s' % j, 3), (-1.0, 'DN2', 3)))
							mdb.models[modelName].Equation(name='3-yz1edge-yz4edge%s' % l,
														   terms=((1.0, 'yz1edge%s' % i, 3), (-1.0, 'yz4edge%s' % l, 3), (-1.0, 'DN2', 3)))

						if x3d_displacement or x3d_displacement or z3d_displacement:
							print("Displacement can only be applied to full pbc models")							
							error3 = 0
							error = True
							message3 = ctypes.windll.user32.MessageBoxA
							message3(error3, 'Displacement can only be applied to full pbc models','Error', 0x30 | 0x0)
							continue
							
					if x3d_axis:
						for i, k in zip(xfront, xback):
							mdb.models[modelName].Equation(name='1-xfront-xback%s' % i,
														   terms=((1.0, 'xfront%s' % i, 1), (-1.0, 'xback%s' % k, 1), (-1.0, 'DN3', 1)))
						for i, k in zip(xfront, xback):
							mdb.models[modelName].Equation(name='2-xfront-xback%s' % i,
														   terms=((1.0, 'xfront%s' % i, 2), (-1.0, 'xback%s' % k, 2), (-1.0, 'DN4', 2)))
						for i, k in zip(xfront, xback):
							mdb.models[modelName].Equation(name='3-xfront-xback%s' % i,
														   terms=((1.0, 'xfront%s' % i, 3), (-1.0, 'xback%s' % k, 3), (-1.0, 'DN5', 3)))

						mdb.models[modelName].Equation(name='1-c34', terms=((1.0, 'c3', 1), (-1.0, 'c4', 1), (1.0, 'DN3', 1)))
						mdb.models[modelName].Equation(name='1-c12', terms=((1.0, 'c1', 1), (-1.0, 'c2', 1), (-1.0, 'DN3', 1)))
						mdb.models[modelName].Equation(name='1-c78', terms=((1.0, 'c7', 1), (-1.0, 'c8', 1), (1.0, 'DN3', 1)))

						mdb.models[modelName].Equation(name='2-c34', terms=((1.0, 'c3', 2), (-1.0, 'c4', 2), (1.0, 'DN4', 2)))
						mdb.models[modelName].Equation(name='2-c12', terms=((1.0, 'c1', 2), (-1.0, 'c2', 2), (-1.0, 'DN4', 2)))
						mdb.models[modelName].Equation(name='2-c78', terms=((1.0, 'c7', 2), (-1.0, 'c8', 2), (1.0, 'DN4', 2)))

						mdb.models[modelName].Equation(name='3-c34', terms=((1.0, 'c3', 3), (-1.0, 'c4', 3), (1.0, 'DN5', 3)))
						mdb.models[modelName].Equation(name='3-c12', terms=((1.0, 'c1', 3), (-1.0, 'c2', 3), (-1.0, 'DN5', 3)))
						mdb.models[modelName].Equation(name='3-c78', terms=((1.0, 'c7', 3), (-1.0, 'c8', 3), (1.0, 'DN5', 3)))

						for i, k, j, l in zip(xy1edge, xy2edge, xy3edge, xy4edge):
							mdb.models[modelName].Equation(name='1-xy1edge-xy2edge%s' % i,
														   terms=((1.0, 'xy1edge%s' % i, 1), (-1.0, 'xy2edge%s' % k, 1), (-1.0, 'DN3', 1)))
							mdb.models[modelName].Equation(name='1-xy3edge-xy4edge%s' % j,
														   terms=((1.0, 'xy3edge%s' % j, 1), (-1.0, 'xy4edge%s' % l, 1), (1.0, 'DN3', 1)))
						for i, k, j, l in zip(xy1edge, xy2edge, xy3edge, xy4edge):
							mdb.models[modelName].Equation(name='2-xy1edge-xy2edge%s' % i,
														   terms=((1.0, 'xy1edge%s' % i, 2), (-1.0, 'xy2edge%s' % k, 2), (-1.0, 'DN4', 2)))
							mdb.models[modelName].Equation(name='2-xy3edge-xy4edge%s' % j,
														   terms=((1.0, 'xy3edge%s' % j, 2), (-1.0, 'xy4edge%s' % l, 2), (1.0, 'DN4', 2)))
						for i, k, j, l in zip(xy1edge, xy2edge, xy3edge, xy4edge):
							mdb.models[modelName].Equation(name='3-xy1edge-xy2edge%s' % i,
														   terms=((1.0, 'xy1edge%s' % i, 3), (-1.0, 'xy2edge%s' % k, 3), (-1.0, 'DN5', 3)))
							mdb.models[modelName].Equation(name='3-xy3edge-xy4edge%s' % j,
														   terms=((1.0, 'xy3edge%s' % j, 3), (-1.0, 'xy4edge%s' % l, 3), (1.0, 'DN5', 3)))
						for i, k, j, l in zip(xz1edge, xz2edge, xz3edge, xz4edge):
							mdb.models[modelName].Equation(name='1-xz1edge-xz2edge%s' % i,
														   terms=((1.0, 'xz1edge%s' % i, 1), (-1.0, 'xz2edge%s' % k, 1), (-1.0, 'DN3', 1)))
							mdb.models[modelName].Equation(name='1-xz3edge-xz4edge%s' % j,
														   terms=((1.0, 'xz3edge%s' % j, 1), (-1.0, 'xz4edge%s' % l, 1), (1.0, 'DN3', 1)))
						for i, k, j, l in zip(xz1edge, xz2edge, xz3edge, xz4edge):
							mdb.models[modelName].Equation(name='2-xz1edge-xz2edge%s' % i,
														   terms=((1.0, 'xz1edge%s' % i, 2), (-1.0, 'xz2edge%s' % k, 2), (-1.0, 'DN4', 2)))
							mdb.models[modelName].Equation(name='2-xz3edge-xz4edge%s' % j,
														   terms=((1.0, 'xz3edge%s' % j, 2), (-1.0, 'xz4edge%s' % l, 2), (1.0, 'DN4', 2)))
						for i, k, j, l in zip(xz1edge, xz2edge, xz3edge, xz4edge):
							mdb.models[modelName].Equation(name='3-xz1edge-xz2edge%s' % i,
													   terms=((1.0, 'xz1edge%s' % i, 3), (-1.0, 'xz2edge%s' % k, 3), (-1.0, 'DN5', 3)))
							mdb.models[modelName].Equation(name='3-xz3edge-xz4edge%s' % j,
													   terms=((1.0, 'xz3edge%s' % j, 3), (-1.0, 'xz4edge%s' % l, 3), (1.0, 'DN5', 3)))

						if x3d_displacement or x3d_displacement or z3d_displacement:
							print("Displacement can only be applied to full pbc models")
							error3 = 0
							error = True
							message3 = ctypes.windll.user32.MessageBoxA
							message3(error3, 'Displacement can only be applied to full pbc models','Error', 0x30 | 0x0)
							continue
