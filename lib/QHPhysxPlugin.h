//---------------------------------------------------------------------
//
// Copyright 2009, SensAble Technologies, Inc.
//
// File: QHPhysxPlugin.h
//
// Author: Venkat Gourishankar	
//
// Created: 12/28/2009
//
// Description: The QHPhysxPlugin class interfaces QH with Nvidia Physx 
// for rendering physics in the uAPI
//
//---------------------------------------------------------------------
#ifndef QHPHYSXPLUGIN_H_
#define QHPHYSXPLUGIN_H_

#include "PhysxPluginCommon.h"

#define FIXED_TIME_STEP 1/60.0

class QHPhysxPlugin
{
private:
	/// Physics SDK globals
	NxPhysicsSDK*     qhPhysicsSDK;
	NxScene*          qhScene;
	/// Tag to enable hardware scene if physx card is present
	bool bHardwareScene;

	/////Physx objects
	//std::vector<NxActor*> m_SphereList;
	//std::vector<NxActor*> m_BoxList;
	//std::vector<NxActor*> m_StaticMeshList;
	//std::vector<NxActor*> m_DynamicMeshList;

	///Haptic LINEAR Virtual coupling parameters
	///Stiffness of virtual coupling (LINEAR SPRING)
	hduVector3Dd K_VirtualLinearCoupling;
	///Damping ratio of virtual coupling (LINEAR SPRING)
	///Instability may result from underdamping. 
	hduVector3Dd C_VirtualLinearCoupling;
	///mass of virtual coupling 
	float M_VirtualLinearCoupling;

	///Haptic TORSIONAL Virtual coupling parameters
	///Stiffness of virtual coupling (LINEAR SPRING)
	hduVector3Dd K_VirtualTorsionCoupling;
	///Damping ratio of virtual coupling (LINEAR SPRING)
	///Haptic Instability may result from underdamping or Overdamping.
	hduVector3Dd C_VirtualTorsionCoupling;
	///mass of virtual coupling 
	float M_VirtualTorsionCoupling;

	float gDeltaTime;

	void StartPhysics(float deltaT);
	void GetPhysicsResults();

	NxMat34 convertQhToPhysxTransform(hduMatrix transform);
	hduMatrix convertPhysxToQhTransform(NxActor* actor);

public:
	//Default constructor
	QHPhysxPlugin();
	///Default simulation parameter
	hduVector3Dd qhDefaultGravity;
	float qhSkinWidth;
	float qhCCDEpsilon;
	bool qhCCDOn;
	NxMaterial* defaultMaterial;

	///Custom constructor
	QHPhysxPlugin(hduVector3Dd gravity, float skin_width, NxMaterial* newMaterial ,float ccdEpsilon);
	///Destructor
	~QHPhysxPlugin();
	
	void stepPhysx();
	void stepPhysx(float deltaT);
	
	NxActor* addSphere(Sphere* qhSphere , const float density);
	NxActor* addBox(Box* qhBox, const float density);
	NxActor* addStaticMesh(TriMesh* qhTrimesh);
	NxActor* addDynamicMesh(TriMesh* qhTrimesh, const float density);
	NxConvexShapeDesc* addConvexHull(const unsigned int vCount, hduVector3Dd* vertices);
	NxActor* addHullsToDynamicMesh(TriMesh* qhTrimesh, NxActorDesc *hullGroupDesc, const float density);

	void updatePhysxActor(Sphere* qhSphere);
	void updatePhysxActor(Box* qhBox);
	void updatePhysxActor(TriMesh* qhTrimesh);
	
	void applyForce(Sphere* qhSphere, hduVector3Dd pushForce, float forceScale);
	void applyForce(Box* qhBox, hduVector3Dd pushForce, float forceScale);
	void applyForce(TriMesh* qhTrimesh, hduVector3Dd pushForce, float forceScale);
	
	void applyTorque(Sphere* qhSphere, hduVector3Dd pushTorque, float torqueScale);
	void applyTorque(Box* qhBox, hduVector3Dd pushTorque, float torqueScale);
	void applyTorque(TriMesh* qhTrimesh, hduVector3Dd pushTorque, float torqueScale);
	
	hduVector3Dd calculateVirtualCouplingForce(Sphere* qhSphere, float stiffness, float dampingRatio,
		hduMatrix deviceToWorld, hduVector3Dd proxyPosition);
	hduVector3Dd calculateVirtualCouplingForce(Box* qhBox, float stiffness, float dampingRatio,
		hduMatrix deviceToWorld, hduVector3Dd proxyPosition);
	hduVector3Dd calculateVirtualCouplingForce(TriMesh* qhTrimesh, float stiffness, float dampingRatio,
		hduMatrix deviceToWorld, hduVector3Dd proxyPosition);

	hduVector3Dd calculateVirtualCouplingTorque(Sphere* qhSphere, float stiffness, float dampingRatio,
		hduMatrix deviceToWorld, hduQuaternion proxyRotation);
	hduVector3Dd calculateVirtualCouplingTorque(Box* qhBox, float stiffness, float dampingRatio,
		hduMatrix deviceToWorld, hduQuaternion proxyRotation);
	hduVector3Dd calculateVirtualCouplingTorque(TriMesh* qhTrimesh, float stiffness, float dampingRatio,
		hduMatrix deviceToWorld, hduQuaternion proxyRotation);

	void initContactRepForPair(TriMesh* qhTrimesh1, TriMesh* qhTrimesh2);
	bool reportContacts();
	NxRevoluteJoint* CreateRevoluteJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis);
	NxPrismaticJoint* CreatePrismaticJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis);
	NxPointOnLineJoint* CreatePointOnLineJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis);

};

#endif