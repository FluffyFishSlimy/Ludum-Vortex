var Module 		= 	{ 
						TOTAL_MEMORY: 512*512*512, 
						//ALLOW_MEMORY_GROWTH:1,
						ABORTING_MALLOC:0,
						ASSERTIONS:1 
					};

var objects 	= 	{};	//objects by gecko id
var bodies		=	{}; //objects by ammo id
var solids 		= 	{};
var softs 		= 	[];
var constraints = 	[];
var vehicles	=	{};

var softPoints=0;

var timeStep = 1/60;
var timerStep = timeStep * 1000;

var substep = 2;//4//3;// default is 1. 2 or more make simulation more accurate.

var Ammo;

var world; 

var pause;

var BUFFER_SOLIDS, BUFFER_SOFTS, BUFFER_COLLISIONS, BUFFER_VEHICLES;

var engine = 
{
	receive 			:	function ( message ) 
							{
								var data = message.data;
								
								if (engine[data.event]) engine[data.event](data);
							},

	send 				: 	function ( type , data , buffers)
							{
								data = data || {};
								data.event = type;
								
								try { self.postMessage ( data , buffers ) } catch (e) {};
							},

	init				:	function init ( properties ) 
							{
							
								importScripts( properties.lib );
							
								Ammo(Module).then
								( 
									function( Ammo ) 
									{ 
							
										initMath();
										engine.initWorld( properties );
								
										engine.send('init');
									}
								);
								
							},

	step				:	function step ( properties )
							{
							
								if ( pause ) return;
								
								
								if (properties.transforms)
								{
									properties.transforms.forEach(function(physics)
									{
										var object = objects[physics.id];
										if (object)
										{
											engine.transform(physics);
										}
									});
								}
								
								world.stepSimulation( timeStep, substep );
								
								if (self.vehicles)
								{
									var currentVehicle = Object.keys(self.vehicles)[0];
									this.driveVehicle(currentVehicle);
								}
													
								this.stepSolidBodies();
								this.stepSoftBodies();
								this.stepCollisions(); //todo use property flag to exclude this step for speed
								this.stepVehicles();
								
								var transforms 	=	BUFFER_SOLIDS 		|| 	new Float32Array();
								var points		= 	BUFFER_SOFTS  		|| 	new Float32Array();
								var collisions	= 	BUFFER_COLLISIONS  	|| 	new Float32Array();
								var vehicles	= 	BUFFER_VEHICLES  	|| 	new Float32Array();
								
								var data		=	{ 
														collisions	: 	collisions.buffer, 
														transforms	: 	transforms.buffer, 
														points		: 	points.buffer,
														vehicles	:	vehicles.buffer,
													};
								
								var buffers		=	[
														collisions.buffer,
														transforms.buffer, 
														points.buffer,
														vehicles.buffer
													];

								engine.send( 'updateWorld' , data ,  buffers);

							},
							
	add					:	function ( properties ) 
							{
								if (!world) return;
								
								switch (properties.type)
								{
									case 'Anchor':
									{
										engine.addAnchor(properties);
										break;
									}
									case 'PointConstraint':
									case 'HingeConstraint':
									case 'SliderConstraint':
									case 'SpringConstraint':
									case 'GenericConstraint':
									{
										engine.addConstraint(properties);
										break;
									}
									case 'Vehicle':
									{
										engine.addVehicle(properties);
										break;
									}
									default :
									{	
										switch (properties.shape/* || 'Box'*/)
										{
											case 'Plane':
											case 'Box':
											case 'Sphere':
											case 'Cylinder':
											case 'Cone':
											case 'Capsule':
											case 'Mesh':
											case 'Convex':
											{
												engine.addSolidBody ( properties ); 
												break;
											}
											case 'Rope':
											case 'Cloth':
											case 'SoftMesh':
											{
												engine.addSoftBody ( properties );
												break;
											}
										}
									}
								}
								
							},

	update				:	function ( properties ) 
							{
								if (!world) return;
								
								if (solids[properties.id]) this.updateSolidBody(properties);
							},
							
	remove				:	function ( properties ) 
							{
								if (!world) return;
								
								if (solids[properties.id]) this.removeSolidBody(properties);
								
							},
							
	reset				:	function reset ( properties ) 
							{
								
								var solidIds=Object.keys(solids);
								solidIds.forEach(function(id)
								{
									var b = solids[id];
									world.removeRigidBody( b );
									Ammo.destroy( b );
									delete solids[id];
								});
							
								while( softs.length > 0)
								{
							
									var b = softs.pop();
									world.removeSoftBody( b );
									Ammo.destroy( b );
							
									softPoints=0;
								}

								while( constraints.length > 0)
								{
							
									var c = constraints.pop();
									world.removeConstraint( c );
									Ammo.destroy( c );
							
								}
							
								objects = {};
								bodies = {};
								solids 	= {};
								softs = [];
								achors=[];
								constraints=[];
								
								engine.clearWorld();
								engine.initWorld();
							
							},
							
	transform			:	function (properties)
							{
								engine.applyMatrix(objects[properties.id], properties.position, properties.rotation);
							},

	applyEnergy			: 	function ( properties )
							{
								var object = objects[properties.id];
								
								if (object)
								{
									var energy = new Ammo.btVector3().fromArray( properties.energy || [0,0,0] );
									var offset  = new Ammo.btVector3().fromArray( properties.offset || [0,0,0] );
									
									switch (properties.type)
									{
										case 'Force'			: 	object.applyForce(energy, offset); break;
										case 'Impulse'			: 	object.applyImpulse(energy, offset); break;
										case 'Torque'			: 	object.applyTorque(energy); break;
										case 'LocalTorque' 		: 	object.applyLocalTorque( energy ); break;
										case 'TorqueImpulse'	: 	object.applyTorque(energy); break;
										case 'CentralImpulse'	: 	object.applyCentralImpulse(energy); break;
										case 'CentralForce' 	:	object.applyCentralForce( energy ); break;
										case 'CentralLocalForce': 	object.applyCentralLocalForce( energy ); break;
									}
									object.activate(true);
								}
								
							},
							
	setVelocity			:	function ( properties ) 
							{
								
								if (properties.linear) setVelocity('LinearVelocity', properties.linear);
								if (properties.angular) setVelocity('AngularVelocity', properties.angular);
								
								function setVelocity(type, velocity)
								{
									objects[properties.id]['set'+type] ( new Ammo.btVector3().fromArray(velocity) );
								}
								
								/*
								if (properties.linear && properties.linear[0]==0 && properties.linear[1]==0 && properties.linear[2]==0)
								{
									objects[properties.id].setActivationState(0);
								}
								else
								{
									objects[properties.id].applyCentralImpulse(new Ammo.btVector3(0,0,0));
									objects[properties.id].activate();
								}
								*/
								objects[properties.id].activate();
							},							

							//--------------------------------------------------
							//
							//  WORLD
							//
							//--------------------------------------------------

	initWorld			:	function initWorld ( properties ) 
							{
							
								if ( world ) return;
							
								properties = properties || {};
							
								var isSoft 			= 	properties.softBodies ? properties.softBodies : true ;
								var solver 		 	= 	new ( Ammo.btSequentialImpulseConstraintSolver ) ();
								var solverSoft 		= 	new ( isSoft ? Ammo.btDefaultSoftBodySolver : {} ) ();
								var collision 		= 	new ( isSoft ? Ammo.btSoftBodyRigidBodyCollisionConfiguration : Ammo.btDefaultCollisionConfiguration) ();
								var dispatcher 	 	= 	new ( Ammo.btCollisionDispatcher ) ( collision );
								var broadphase 	 	= 	function ()
														{
															switch( properties.broadphase || 'Dynamic' )
															{
																case 'Dynamic':
																case 'AxisSweep'	: return new Ammo.btAxisSweep3( new Ammo.btVector3(-1000,-1000,-1000), new Ammo.btVector3(1000,1000,1000), 4096 ); 
																//case 'Dynamic'		: return new Ammo.btDbvtBroadphase(); 
															}
														}();
														
							
								world		 		= 	new (isSoft ? Ammo.btSoftRigidDynamicsWorld : Ammo.btDiscreteDynamicsWorld) ( dispatcher, broadphase, solver, collision, solverSoft );
							
								world.isSoft 		= 	isSoft;
								world.solver 		= 	solver;
								world.solverSoft	= 	solverSoft;
								world.collision 	= 	collision;
								world.dispatcher 	= 	dispatcher;
								world.broadphase 	= 	broadphase;
								
								world.getDispatchInfo().set_m_allowedCcdPenetration(0.001);// default 0.0399
								
								this.setGravity( properties );								
							},

	clearWorld			:	function clearWorld () 
							{
								
								Ammo.destroy( world.solver );
								Ammo.destroy( world.solverSoft );
								Ammo.destroy( world.collision );
								Ammo.destroy( world.dispatcher );
								Ammo.destroy( world.broadphase );	

								Ammo.destroy( world );
							
								world = null;
								
								engine.initWorld();
							
							},
	
	setGravity 			:	function setGravity ( properties ) 
							{
							
								properties = properties || {};
							
								if ( !world ) return;
							
								if ( !world.gravity ) world.gravity = new Ammo.btVector3();
							
								world.gravity.fromArray( properties.gravity || [0,-10, 0] );
								world.setGravity( world.gravity );
								
								if( world.isSoft )
								{
									world.info = world.getWorldInfo();
							
									world.info.set_air_density( properties.airDensity || 1.2 );//1.275
									world.info.set_m_gravity( world.gravity );
									//setWater( properties );
								}
							
							},



							//--------------------------------------------------
							//
							//  ADD
							//
							//--------------------------------------------------
							
	addAnchor			:	function ( properties )
							{
								var source = objects[properties.source];
								var target = objects[properties.target];
								var influence = properties.influence!=undefined ? properties.influence : 0.5
								
								if (!source || !target) return;
								//source.setActivationState( 4 );
								
							    var anchor = source.appendAnchor(properties.point, target, disableCollisionBetweenLinkedBodies=true, influence);
							},
							
	addConstraint		: 	function ( properties )
							{
								
								var body1 = objects[properties.body1];
								var point1 = new Ammo.btVector3().fromArray( properties.point1 || [0,0,0] );
								
								var body2 = objects[properties.body2];
								var point2 = new Ammo.btVector3().fromArray( properties.point2 || [0,0,0] );
								
								var axis1 = new Ammo.btVector3().fromArray( properties.axis1 || [0,1,0] );
								var axis2 = new Ammo.btVector3().fromArray( properties.axis2 || [0,0,0] );
								
								var allowCollisions = properties.collisions==undefined || properties.collisions ? true : false;
								
								if ( properties.type == 'SpringConstraint' || properties.type == 'SliderConstraint' || properties.type=='GenericConstraint')
								{
									var rotation1 = new Ammo.btQuaternion().fromArray( properties.rotation1  || [0,0,0,0] );
	
									var frameA = new Ammo.btTransform();
									frameA.setIdentity();
									frameA.setOrigin( point1 );
									frameA.setRotation( rotation1 );
									
									var rotation2 = new Ammo.btQuaternion().fromArray( properties.rotation2 || [1,0,0,0] );

									var frameB = new Ammo.btTransform();
									frameB.setIdentity();
									frameB.setOrigin( point2 );
									frameB.setRotation( rotation2 );

								}
								
								var constraint;

								switch(properties.type)
								{
									case "PointConstraint":
									{
										if (body2)
										{
											constraint = new Ammo.btPoint2PointConstraint( body1, body2, point1, point2);
										}
										else
										{
											constraint = new Ammo.btPoint2PointConstraint( body1, point1 );
										}
										
										if ( properties.strength != undefined ) constraint.get_m_setting().set_m_tau( properties.strength );
										if ( properties.damping != undefined ) constraint.get_m_setting().set_m_damping( properties.damping ); 
										if ( properties.impulse != undefined ) constraint.get_m_setting().set_m_impulseClamp( properties.impulse );
										
										break;
									}
									case "HingeConstraint":
									{
										if (body2)
										{
											constraint = new Ammo.btHingeConstraint( body1, body2, point1, point2, axis1, axis2, false );
										}
										else
										{
											constraint = new Ammo.btHingeConstraint( body1, point1, axis1 );
										}
										
										if( properties.velocity ) constraint.enableAngularMotor( true, properties.velocity, properties.acceleration!=undefined ? properties.acceleration : 1 );
										if( properties.limits ) constraint.setLimit( properties.limits.min  , properties.limits.max , properties.limits.softness || 0.1, properties.limits.bias || 0.3, properties.limits.restitution || 0.9 );
										
										

										break;
									}
									case "SliderConstraint": //not completed
									{
										constraint = new Ammo.btSliderConstraint( body1, body2, frameA, frameB, useFrameA=false )
									
										if (properties.limits)
										{	
											console.log('constraint', constraint, frameA.getRotation());
											constraint.setLowerLinLimit( 0 );
											constraint.setUpperLinLimit( 5 )
										}

										console.log(constraint);
										break;
									}
									case "SpringConstraint": //not completed
									{
										constraint = new Ammo.btGeneric6DofSpringConstraint( body1, body2, frameA, frameB , useFrameA=false);
									
										if (properties.limits && properties.limits.linear)
										{	
											constraint.setLinearLowerLimit( new Ammo.btVector3().fromArray( properties.limits.linear.min || [0,0,0] ) );
											constraint.setLinearUpperLimit( new Ammo.btVector3().fromArray( properties.limits.linear.max || [0,0,0] ) )
										}
										
										constraint.enableSpring(1,true);
										
										console.log('>>>>', constraint);

										/*
										constraint.setAngularLowerLimit( new Ammo.btQuaternion().fromArray( [0,0,0,1] ) );
										constraint.setAngularUpperLimit( new Ammo.btQuaternion().fromArray( [1,0,0,1] ) )
										
										body1.activate();
										
										constraint.enableFeedback();
										var motor = constraint.getRotationalLimitMotor( 1 );
										motor.set_m_enableMotor( true );
										*/

										
										
										break;
									}
									case "GenericConstraint": //not completed
									{
										constraint = new Ammo.btGeneric6DofConstraint( body1, body2, frameA, frameB , useFrameA=true);
									
										if (properties.limits && properties.limits.linear)
										{	
											constraint.setLinearLowerLimit( new Ammo.btVector3().fromArray( properties.limits.linear.min || [0,0,0] ) );
											constraint.setLinearUpperLimit( new Ammo.btVector3().fromArray( properties.limits.linear.max || [0,0,0] ) )
										}

									
										if (properties.limits && properties.limits.angular)
										{	
											constraint.setAngularLowerLimit( new Ammo.btQuaternion().fromArray( properties.limits.angular.min || [0,0,0,0] ) );
											constraint.setAngularUpperLimit( new Ammo.btQuaternion().fromArray( properties.limits.angular.min || [1,0,0,1] ) )
										}

										/*
										constraint.setAngularLowerLimit( new Ammo.btQuaternion().fromArray( [0,0,0,1] ) );
										constraint.setAngularUpperLimit( new Ammo.btQuaternion().fromArray( [1,0,0,1] ) )
										
										body1.activate();
										
										constraint.enableFeedback();
										var motor = constraint.getRotationalLimitMotor( 1 );
										motor.set_m_enableMotor( true );
										*/

										
										
										break;
									}
								}
								
								
								world.addConstraint( constraint, noAllowCollision=!allowCollisions );

								//setTimeout(function(){console.log('x'); world.removeConstraint(constraint) }, 1600);
							
								constraints.push( constraint );
								
							},
							
	createSolidShape	:	function (properties)
							{
								switch( properties.shape )
								{
									case 'Plane'	: 	return new Ammo.btStaticPlaneShape( new Ammo.btVector3( properties.direction || [0,1,0] ) , 0 );
									case 'Box'		: 	return new Ammo.btBoxShape( new Ammo.btVector3( properties.bounds[0]*0.5, properties.bounds[1]*0.5, properties.bounds[2]*0.5 ) );
									case 'Sphere'	:	return new Ammo.btSphereShape( properties.bounds[0]*0.5 ); 
									case 'Cylinder'	:	return new Ammo.btCylinderShape( new Ammo.btVector3( properties.bounds[0]*0.5, properties.bounds[1]*0.5, properties.bounds[2]*0.5 ) );
									case 'Cone'		: 	return new Ammo.btConeShape( properties.bounds[0]*0.5, properties.bounds[1] );
									case 'Capsule'	: 	return new Ammo.btCapsuleShape( properties.bounds[0]*0.5, properties.bounds[1]-properties.bounds[0] );
									case 'Mesh'		: 	{
															var mesh  = new Ammo.btTriangleMesh();
															var point = new Ammo.btVector3();

															if (!properties.geometry.length) return false;
												
															for ( var i = 0; i < properties.geometry.length; i++ ) 
															{
																var face = properties.geometry[i];
																mesh.addTriangle(point.fromArray(face[0]), point.fromArray(face[1]), point.fromArray(face[2]), true);
															}
															
															return new (properties.mass==0 ? Ammo.btBvhTriangleMeshShape : Ammo.btConvexTriangleMeshShape) (mesh, true, true);
														}
									case 'Convex'	:	{	
															var shape = new Ammo.btConvexHullShape();
															var point = new Ammo.btVector3();

															for ( var i = 0; i < properties.geometry.length; i++ ) 
															{
																shape.addPoint(point.fromArray( properties.geometry[i] ));
															}
															
															return shape;
														}
								}
							},
							
	addSolidBody		:	function ( properties ) 
							{
							
								properties.mass			= 	properties.mass 	|| 0;
								properties.bounds 		= 	properties.bounds 	|| [1,1,1];
								properties.position 	= 	properties.position || [0,0,0];
								properties.rotation 	= 	properties.rotation || [0,0,0,1];
							
								var shape = engine.createSolidShape(properties);

								if (!shape) return;
								
								if (properties.children)
								{
									var compoundShape = new Ammo.btCompoundShape();
									var transform = new Ammo.btTransform();
									transform.setIdentity();
									
									compoundShape.addChildShape( transform, shape );
									
									for ( var i = 0; i < properties.children.length; i++ ) 
									{
										var child = properties.children[i];
										transform.setOrigin(new Ammo.btVector3().fromArray(child.position));  
										transform.setRotation(new Ammo.btQuaternion().fromArray(child.rotation)); //TODO fix bug. If the model geometry was not centered, rotations screw up position
								
										compoundShape.addChildShape( transform, createShape( child ) );
									}
									
									shape = compoundShape;									

									Ammo.destroy(transform);
								}
								
							
								var transform = new Ammo.btTransform();
								transform.setIdentity();
								transform.setOrigin( new Ammo.btVector3().fromArray( properties.position ) );
								transform.setRotation( new Ammo.btQuaternion().fromArray( properties.rotation ) );
							
								var inertia = new Ammo.btVector3(0,0,0);
								shape.calculateLocalInertia( properties.mass, inertia );
							
								var motionState = new Ammo.btDefaultMotionState( transform );
							
								var rbInfo = new Ammo.btRigidBodyConstructionInfo( properties.mass, motionState, shape, inertia );
								rbInfo.set_m_friction( properties.friction || 0.5 );
								rbInfo.set_m_restitution( properties.restitution || 0 );
								
								var body = new Ammo.btRigidBody( rbInfo );
							
							
								if (Array.isArray(properties.mask)) properties.mask = function(){var sum=0; properties.mask.forEach(function(val){sum+=val}); return sum}();
								if ( properties.mass !== 0 )
								{
									body.setCollisionFlags( properties.flags || 0 );
									world.addRigidBody( body, properties.group || 1, properties.mask || -1 );
							
									body.setActivationState( properties.state || 1 );
								} 
								else 
								{
									body.setCollisionFlags(properties.flags || 1); 
									world.addCollisionObject( body, properties.group || 1, properties.mask || -1 );
								}
								
								//body.setCcdMotionThreshold( 100 )
								
							   // if( properties.name ) byName[ properties.name ] = body;
							   // else if ( properties.mass !== 0 ) byName[ bodys.length ] = body;
							
								//if ( properties.mass !== 0 ) bodies.rigid.push( body );
								//else bodies.solid.push( body );
								
								body.id=properties.id;
								body.properties = properties;
								
								solids[body.id]=body;
							
								Ammo.destroy( rbInfo );
								Ammo.destroy( transform );
								
								//properties = null;
								
								objects[body.id]=body;
								bodies[body.a || body.ptr] = body;
								
								return body;
							
							},
							
	updateSolidBody		:	function ( properties )
							{
								var body = objects[properties.id];
								
								// Updating properties like mass and kinematics can be buggy for objects already added to the world. 
								// See http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=&f=9&t=3663#p13816
								world.removeRigidBody(body);
								
								body.setCollisionFlags(properties.flags);
								
								world.addRigidBody(body);
								
								//body.setActivationState( properties.state || 1 );
								
								//body.setMassProps( properties.mass, new Ammo.btVector3().fromArray([0,0,0]) );
								//body.activate();
								
							},

	removeSolidBody		:	function ( properties )
							{
								var body = objects[properties.id];
								
								world.removeRigidBody(body);
								
								Ammo.destroy(body);
								
								delete objects[properties.id];
								delete solids[properties.id];
								delete bodies[body.a];
							},
							
	addVehicle			:	function ( properties ) 
							{
								
								properties.mass 			= 	properties.mass 			|| 	800;
								properties.masscenter 		= 	properties.masscenter 		|| 	[0,0,0];
								properties.bounds 			= 	properties.bounds 			|| 	[2,1,4]; 
								properties.pos 				= 	properties.pos 				|| 	[0,2,0];
								properties.quat 			= 	properties.quat 			|| 	[0,0,0,1];
								
								
								properties.friction			=	properties.friction 		|| 	0.5;
								properties.restitution 		=	properties.restitution 		|| 	0 ;
								properties.linearDamping	=	properties.linearDamping	|| 	0;
								properties.angularDamping	=	properties.angularDamping 	|| 	1;
								
							
								var carInfo = 
								{
									steering:0, 
									engine:0, 
									breaking:0, 
							
									incSteering		: 	properties.incSteering 	|| 0.04, 
									maxSteering		: 	properties.maxSterring 	|| 0.5,//Math.PI/6,
									incEngine		: 	properties.acceleration || 10, 
									maxEngine		: 	properties.engine 		|| 1000,
									maxBreaking 	: 	properties.maxBreaking 	|| 100
								};
							
							
								//----------------------------
								// car shape 
							
								var shape = engine.createSolidShape(Object.assign(properties, {shape:'Convex'}));
							
								//----------------------------
								// move center of mass
							
								var center = new Ammo.btVector3().fromArray( properties.masscenter ).negate();
								var transform = new Ammo.btTransform();
								transform.setIdentity();
								transform.setOrigin( center );
							
								var compound = new Ammo.btCompoundShape();
								compound.addChildShape( transform, shape );
							
							
								//----------------------------
								// position rotation of car 
							
								var position = new Ammo.btVector3().fromArray( properties.pos );
								var rotation = new Ammo.btQuaternion().fromArray( properties.quat );
								
								transform.setIdentity();
								transform.setOrigin( position );
								transform.setRotation( rotation );
							
								//----------------------------
								// physics setting
							
								// mass of vehicle in kg
								var inertia = new Ammo.btVector3(0,0,0);
								compound.calculateLocalInertia( properties.mass, inertia );
							
								var motionState = new Ammo.btDefaultMotionState( transform );
								
								var rbInfo = new Ammo.btRigidBodyConstructionInfo( properties.mass, motionState, compound, inertia );
								rbInfo.set_m_friction( properties.friction );
								rbInfo.set_m_restitution( properties.restitution  );
								rbInfo.set_m_linearDamping( properties.linearDamping  );
								rbInfo.set_m_angularDamping( properties.angularDamping  );
							
								//----------------------------
								// car body
							
								var body = new Ammo.btRigidBody( rbInfo );
							
								
								body.setAngularVelocity( new Ammo.btVector3(0,0,0) );
								body.setLinearVelocity( new Ammo.btVector3(0,0,0) );
								body.setActivationState( 4 );
							
								//----------------------------
								// suspension setting
							
								var tuning = new Ammo.btVehicleTuning();
							
								// 10 = Offroad buggy, 50 = Sports car, 200 = F1 Car
								tuning.set_m_suspensionStiffness( properties.s_stiffness || 20 );
								// The damping coefficient for when the suspension is compressed. Set
								// to k * 2.0 * btSqrt(m_suspensionStiffness) so k is proportional to critical damping.
								// k = 0.0 undamped & bouncy, k = 1.0 critical damping
								// k = 0.1 to 0.3 are good values , default 0.84
								tuning.set_m_suspensionCompression( properties.s_compression || 0.84);//4.4 );
								// The damping coefficient for when the suspension is expanding.
								// m_suspensionDamping should be slightly larger than set_m_suspensionCompression, eg k = 0.2 to 0.5, default : 0.88
								tuning.set_m_suspensionDamping( properties.s_damping || 0.88);//2.3 );
							
								 // The maximum distance the suspension can be compressed in Cm // default 500
								tuning.set_m_maxSuspensionTravelCm( properties.s_travel || 100 );
								// Maximum suspension force
								tuning.set_m_maxSuspensionForce( properties.s_force || 10000 );
								// suspension resistance Length
								// The maximum length of the suspension (metres)
								var s_length = properties.s_length || 0.2;
							
								//console.log('++++',tuning.get_m_suspensionCompression())
							
							
								//suspensionForce = stiffness * (restLength – currentLength) + damping * (previousLength – currentLength) / deltaTime
								// http://www.digitalrune.com/Blog/Post/1697/Car-Physics-for-3D-Games
							
								//----------------------------
								// wheel setting
							
								var radius = properties.radius || 0.4;
								var wPos = properties.wPos || [1, 0, 1.6];
							
								//wPos[1] += properties.masscenter[1];
								wPos[1] -= properties.masscenter[1];
							
								// friction: The constant friction of the wheels on the surface.
								// For realistic TS It should be around 0.8. default 10.5
								// But may be greatly increased to improve controllability (1000 and more)
								// Set large (10000.0) for kart racers
								tuning.set_m_frictionSlip( properties.w_friction || 1000 );
								// roll: reduces torque from the wheels
								// reducing vehicle barrel chance
								// 0 - no torque, 1 - the actual physical behavior
								var w_roll = properties.w_roll || 0.1;
							
								//----------------------------
								// create vehicle
							
								var vehicleRayCaster = new Ammo.btDefaultVehicleRaycaster( world );
								var car = new Ammo.btRaycastVehicle(tuning, body, vehicleRayCaster);
								car.setCoordinateSystem( 0, 1, 2 );
								
								var numWheels = properties.nw || 4, p, fw;
							
								for( var i = 0; i < numWheels; i++ ){
									
									if( i==2 && wPos[4]) wPos[0] += wPos[4]; 
							
									if(i==0){ p = [ wPos[0], wPos[1],  wPos[2] ]; fw = true; }
									if(i==1){ p = [-wPos[0], wPos[1],  wPos[2] ]; fw = true; }
									if(i==2){ p = [-wPos[0], wPos[1], -wPos[2] ]; fw = false; }
									if(i==3){ p = [ wPos[0], wPos[1], -wPos[2] ]; fw = false; }
									if(i==4){ p = [-wPos[0], wPos[1], -wPos[3] ]; fw = false; }
									if(i==5){ p = [ wPos[0], wPos[1], -wPos[3] ]; fw = false; }
							
									if( numWheels == 2 )
									{ // moto
										if(i==1){ p = [ -wPos[0], wPos[1],  -wPos[2] ]; fw = false; }
									}
							
									addWheel( car, p, radius, tuning, s_length, w_roll, fw );
							
									car.setBrake( carInfo.maxBreaking, i );
								
								};
							
								world.addAction( car );
								world.addRigidBody( body );
							
							
								body.activate();
								
								car.id=properties.id;
								car.info = carInfo;
								body.id=properties.id;
								
								vehicles[car.id]=car;
								
								objects[body.id]=body;
								bodies[body.a || body.ptr] = body;

								Ammo.destroy( rbInfo );
								Ammo.destroy( transform );
								
								properties = null;

								function addWheel ( car, pos, radius, tuning, s_length, w_roll, isFrontWheel ) 
								{
								
									var position = new Ammo.btVector3().fromArray( pos ); // position
									var wheelDir = new Ammo.btVector3(); wheelDir.setValue( 0,-1,0 ); // wheelDir
									var wheelAxe = new Ammo.btVector3(); wheelAxe.setValue( -1,0,0 ); // wheelAxe
								
									var wheel = car.addWheel( position, wheelDir, wheelAxe, s_length, radius, tuning, isFrontWheel );
									wheel.set_m_rollInfluence( w_roll );
								
									//wheel.set_m_frictionSlip(tuning.get_m_frictionSlip());
								
								
									//console.log('????', wheel.get_m_raycastInfo().get_m_suspensionLength())
								
								};

							},
							
	driveVehicle		:	function ( id ) 
							{
							
								id = id || 0;
								if( !self.vehicles[id] ) return;
							
								var car = vehicles[id];
								var u = car.info;
								var wn = car.getNumWheels();
								
								
								
								u.steering -= u.incSteering * 1 /*key[0]*/;
							
								if( u.steering < -u.maxSteering ) u.steering = -u.maxSteering;
								if( u.steering > u.maxSteering ) u.steering = u.maxSteering;
							
								u.steering *= 0.9;
							
								u.engine -= u.incEngine * -1 /*key[1]*/;
								if( u.engine > u.maxEngine ) u.engine = u.maxEngine;
								if( u.engine < -u.maxEngine ) u.engine = -u.maxEngine;
								
								
								if( 1/*key[1]*/ == 0  )
								{
									if( u.engine > 1 ) u.engine *= 0.9;
									else if ( u.engine < -1 ) u.engine *= 0.9;
									else { u.engine = 0; u.breaking = u.maxBreaking; }
								}
							
								var i = wn;
								while(i--)
								{
									if( i == 0 ) car.setSteeringValue( u.steering, i );
									if(wn !== 2 && i == 1 ) car.setSteeringValue( u.steering, i );
									car.applyEngineForce( u.engine, i );
									car.setBrake( u.breaking, i );
								}
								
								//car.getRigidBody().setAngularVelocity( new Ammo.btVector3(0,u.steering,0) );

								
							
							},
							
	stepVehicles		:	function ()
							{
								
								var collection = Object.keys(vehicles);
								
								BUFFER_VEHICLES =  new TransferArray( collection.length , 8+28 ); //  Speed(1) + Position(3) + Rotation(4) + wheels(4*(3+4)=)
								
								collection.forEach(function(key, id)
								{
									var vehicle = vehicles[key];
									
									var offset = id*BUFFER_VEHICLES[0] + 1;
									
									BUFFER_VEHICLES[offset+0]=vehicle.getCurrentSpeedKmHour();

									var transform = new Ammo.btTransform();
									vehicle.getRigidBody().getMotionState().getWorldTransform( transform );
									transform.toArray( BUFFER_VEHICLES, offset+1 );									
								
									j = vehicle.getNumWheels(); //2, 4 or 6;
							
									while(j--)
									{
										vehicle.updateWheelTransform( j, true );
										var transform = vehicle.getWheelTransformWS( j );
							
										var w = 7 * j ;
										transform.toArray( BUFFER_VEHICLES, offset + w + 8 );
									   
										//if( j === 0 ) AR[ n + w ] = b.getWheelInfo(0).get_m_steering();
							
									}	
									
								});
								
								
								
							},
												
							
	addSoftBody			:	function ( properties ) 
							{
							
								properties.size = properties.size || [1,1,1] ;
								properties.divisions = properties.divisions || [64,64] ;
								properties.position = properties.position || [0,0,0];
								
								var softBodyHelpers = new Ammo.btSoftBodyHelpers();
							
								var body;
								
								var self = this;
								
								switch( properties.shape )
								{
									case 'Cloth':
									{
										var w = properties.size[0] * 0.5;
										var h = properties.size[2] * 0.5;
										var p = properties.pins || [1,1,1,1];
										var dx = properties.divisions[0];
										var dy = properties.divisions[1];
										var x = properties.position[0];
										var y = properties.position[1]; 
										var z = properties.position[2];
										
										/*
										*
										*  corners:
										*
										*  		corner0 ------- corner1  
										*          |               |
										*          |               |
										*  		corner2 -------- corner3  
										*
										*/										
										var corner0 = new Ammo.btVector3().fromArray( [ -w+x, y, -h+z ] );
										var corner1 = new Ammo.btVector3().fromArray( [  w+x, y, -h+z ] );
										var corner2 = new Ammo.btVector3().fromArray( [ -w+x, y,  h+z ] );
										var corner3 = new Ammo.btVector3().fromArray( [  w+x, y,  h+z ] );
										
										body = softBodyHelpers.CreatePatch( world.info, corner0, corner1, corner2, corner3, dx, dy, getFixedPoints(p), properties.gendiags  );
										body.softType = 1;
										
										softPoints+=properties.points;
										
										break;
									}
									case 'Rope':
									{
										var start = new Ammo.btVector3().fromArray(properties.start || [10,0,0]);
										var end = new Ammo.btVector3().fromArray(properties.end || [-10,0,0]);
										var pins = properties.pins || [1,1];
										var segments = (properties.segments || 10);
										//properties.margin = (properties.radius || 0.2);//*2;
							
										body = softBodyHelpers.CreateRope( world.info, start, end, segments-2, getFixedPoints(pins), true);
										
										softPoints+=properties.points; //body.get_m_nodes().size();
										
										body.softType = 2;
										break;
									}
									case 'SoftMesh':
									{
										var vertices = properties.vertices;
										var indices = properties.indices;
										var faces = properties.faces;
										
										body = softBodyHelpers.CreateFromTriMesh( world.info, vertices, indices, faces, randomizeConstraints = properties.random || true );
										body.softType = 5;	

										softPoints+=properties.points; //body.get_m_nodes().size();
										break;									
									}
								}
							
								var sb = body.get_m_cfg();
							
								setIterations(body, properties.iterations);
								setStiffness(body, properties.stiffness);
							
								sb.set_collisions( 0x11 );
							
								if( properties.friction !== undefined ) sb.set_kDF(properties.friction);
								if( properties.damping !== undefined ) sb.set_kDP(properties.damping);
								if( properties.pressure !== undefined ) sb.set_kPR(properties.pressure);
								if( properties.viscosity !== undefined ) sb.set_kVC(properties.viscosity);
							
								body.setTotalMass( properties.mass, properties.fromfaces || false );
								body.setActivationState( properties.state || 1 );								

								if(properties.margin !== undefined ) Ammo.castObject( body, Ammo.btCollisionObject ).getCollisionShape().setMargin( properties.margin );
							
							
								// Soft-soft and soft-rigid collisions
								world.addSoftBody( body, properties.group || 1, properties.mask || -1 );
							
								body.points = body.get_m_nodes().size();
							
								//if( properties.name ) byName[properties.name] = body;
								
								body.id=properties.id;
								body.properties = properties;
							
								softs.push( body );
								
								objects[body.id] = body;
								bodies[body.a || body.ptr] = body;
							
								Ammo.destroy(softBodyHelpers);
								
								return body;
								
								function setIterations(body, properties)
								{
									setConfiguration(body.get_m_cfg(), 'set_', ['v','p','c','d'], 'iterations', properties );
								}
								
								function setStiffness(body, properties)
								{
									setConfiguration(body.get_m_materials().at(0), 'set_m_k', ['L','A','V'], 'ST', properties );
								}	
								
								function setConfiguration(object, prefix, letters, postfix, properties)
								{
									var result=[];
									
									var isArray = Array.isArray(properties);
									var isObject = properties instanceof Object;
									
									letters.forEach( function( letter, index)
									{
										var i = isObject ? properties[ isArray ? index : letter.toLowerCase()] : properties;
										if (i!=undefined) object[prefix+letter+postfix]( i );
									});
								}
								
								function getFixedPoints(pins)
								{
									var result=0;
									
									for (var i=0; i<pins.length; i++) result+=pins[i]*Math.pow(2,i);
										
									return result;
									
								}
							},
							
	stepSolidBodies		:	function ()
							{
								var collection = Object.keys(solids);
								
								BUFFER_SOLIDS =  new TransferArray( collection.length , 13 ); //  Position(3) + Rotation(4) + LinearVelocity(3) + AngularVelocity(3);
								
								collection.forEach(function(key, id)
								{
									var body = solids[key];
									
									var transform = new Ammo.btTransform();
									body.getMotionState().getWorldTransform( transform );
									
									transform.toArray( BUFFER_SOLIDS, 1 + id*BUFFER_SOLIDS[0] );
									
									var linearVelocity = body.getLinearVelocity();
									linearVelocity.toArray( BUFFER_SOLIDS, 8 + id*BUFFER_SOLIDS[0] );
									
									var angularVelocity = body.getAngularVelocity();
									linearVelocity.toArray( BUFFER_SOLIDS, 11 + id*BUFFER_SOLIDS[0] );
								});
							},
					
	stepSoftBodies		:	function ()
							{
								if( !softs.length ) return;
								
								var points = [3];
							
								softs.forEach( function ( body ) 
								{
									var nodes = body.get_m_nodes(); 
									var j = nodes.size();
									
									var n, p = points.length;
									
									while(j--)
									{
										n = p + ( j * 3 );
										nodes.at( j ).get_m_x().toArray( points , n );
									}
								});
								
								BUFFER_SOFTS = new TransferArray( points , points[0] );								
							},
							
	stepCollisions		:	function ()
							{
								var manifolds = world.dispatcher.getNumManifolds();
								
								//console.log('----------------->', 'manifolds', manifolds);	
								
								var collisions = [5];
								
								for ( var i = 0; i < manifolds; i++ ) 
								{
									var manifold = world.dispatcher.getManifoldByIndexInternal( i );
							
									var contacts = manifold.getNumContacts();
									
									var body0 = manifold.getBody0();
									var body1 = manifold.getBody1();
									
									//console.log('::---------', i, 'contacts=', contacts, "body0=", bodies[body0.a], "body1", bodies[body1.a]);
									
									for ( var j = 0; j < contacts; j++ ) 
									{
										var point = manifold.getContactPoint( j );
										
										var offset = collisions.length;
										
										collisions[ offset + 0 ] = bodies[body0.a].id;
										collisions[ offset + 1 ] = bodies[body1.a].id;
						
										var normal = point.get_m_normalWorldOnB();
										collisions[ offset + 2 ] = normal.x();
										collisions[ offset + 3 ] = normal.y();
										collisions[ offset + 4 ] = normal.z();
										
										/*
										var pointA = point.get_m_localPointA();
										collisions[ offset + 5 ] = pointA.x();
										collisions[ offset + 6 ] = pointA.y();
										collisions[ offset + 7 ] = pointA.z();
										
										var pointB = point.get_m_localPointB();
										collisions[ offset + 8 ] = pointB.x();
										collisions[ offset + 9 ] = pointB.y();
										collisions[ offset + 10 ] = pointB.z();
										*/
							
									}
									
								}		
								
								//console.log('.............', collisions);						
								
								BUFFER_COLLISIONS = new TransferArray(collisions, collisions[0]);
							},



							
//--------------------------------------------------
//
//  KINEMATICS MATRIX SET
//
//--------------------------------------------------

	applyMatrix 		: 	function  ( object , position, rotation ) 
							{
							
								if ( !object ) return;
								
								var transform = new Ammo.btTransform();
							
								transform.setIdentity();
							
								if( position ) {  transform.setOrigin( new Ammo.btVector3().fromArray( position ) ); }
								if( rotation ) { transform.setRotation( new Ammo.btQuaternion().fromArray( rotation ) ); }
								

								//console.log('>>>>', object, position, rotation);

								object.getMotionState().setWorldTransform( transform );
								
								object.position = position;
								object.rotation = rotation;
								
								Ammo.destroy(transform);
							
							}												

}

//--------------------------------------------------
//
//  AMMO MATH
//
//--------------------------------------------------

var torad = 0.0174532925199432957;
var todeg = 57.295779513082320876;

//--------------------------------------------------
//
//  btTransform extend
//
//--------------------------------------------------

function initMath()
{

    Ammo.btTransform.prototype.toArray = function( array, offset )
	{

        //if ( offset === undefined ) offset = 0;
        offset = offset || 0;

        this.getOrigin().toArray( array , offset );
        this.getRotation().toArray( array , offset + 3 );

        //return array;

    };

    //--------------------------------------------------
    //
    //  btVector3 extend
    //
    //--------------------------------------------------

    Ammo.btVector3.prototype.negate = function( v )
	{

        this.setValue( -this.x(), -this.y(), -this.z() );
        return this;

    };

    Ammo.btVector3.prototype.add = function( v )
	{

        this.setValue( this.x() + v.x(), this.y() + v.y(), this.z() + v.z() );
        return this;

    };

    Ammo.btVector3.prototype.fromArray = function( array, offset )
	{

        //if ( offset === undefined ) offset = 0;
        offset = offset || 0;

        this.setValue( array[ offset ], array[ offset + 1 ], array[ offset + 2 ] );

        return this;

    };

    Ammo.btVector3.prototype.toArray = function( array, offset )
	{

        //if ( array === undefined ) array = [];
        //if ( offset === undefined ) offset = 0;
        offset = offset || 0;

        array[ offset ] = this.x();
        array[ offset + 1 ] = this.y();
        array[ offset + 2 ] = this.z();

        //return array;

    };

    Ammo.btVector3.prototype.direction = function( q )
	{

        // quaternion 
        
        var qx = q.x();
        var qy = q.y();
        var qz = q.z();
        var qw = q.w();

        var x = this.x();
        var y = this.y();
        var z = this.z();

        // calculate quat * vector

        var ix =  qw * x + qy * z - qz * y;
        var iy =  qw * y + qz * x - qx * z;
        var iz =  qw * z + qx * y - qy * x;
        var iw = - qx * x - qy * y - qz * z;

        // calculate result * inverse quat

        var xx = ix * qw + iw * - qx + iy * - qz - iz * - qy;
        var yy = iy * qw + iw * - qy + iz * - qx - ix * - qz;
        var zz = iz * qw + iw * - qz + ix * - qy - iy * - qx;

        this.setValue( xx, yy, zz );

    };

    //--------------------------------------------------
    //
    //  btQuaternion extend
    //
    //--------------------------------------------------

    Ammo.btQuaternion.prototype.fromArray = function( array, offset )
	{

        //if ( offset === undefined ) offset = 0;
        offset = offset || 0;
        this.setValue( array[ offset ], array[ offset + 1 ], array[ offset + 2 ], array[ offset + 3 ] );
		
		return this;

    };

    Ammo.btQuaternion.prototype.toArray = function( array, offset )
	{

        //if ( array === undefined ) array = [];
        //if ( offset === undefined ) offset = 0;
        offset = offset || 0;

        array[ offset ] = this.x();
        array[ offset + 1 ] = this.y();
        array[ offset + 2 ] = this.z();
        array[ offset + 3 ] = this.w();

        //return array;

    };

    Ammo.btQuaternion.prototype.setFromAxisAngle = function( axis, angle )
	{

        var halfAngle = angle * 0.5, s = Math.sin( halfAngle );
        this.setValue( axis[0] * s, axis[1] * s, axis[2] * s, Math.cos( halfAngle ) );

    };

}

function TransferArray ( arrayOrLength, chunksize )
{
	if (Array.isArray(arrayOrLength))
	{
		array = new Float32Array( arrayOrLength );
	}
	else
	{
		var length = arrayOrLength;
		array = new Float32Array( new ArrayBuffer( (1 + length*chunksize) * Float32Array.BYTES_PER_ELEMENT ) );
	}
	
	array[0]=chunksize;
	return array;
};

self.onmessage = engine.receive;

