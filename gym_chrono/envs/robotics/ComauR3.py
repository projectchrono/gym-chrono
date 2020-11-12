# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\SB\Documents\CAD\Comau-R3\Racer3 (3D-SOLIDWORKS)\Racer3_HandE.SLDASM


import pychrono as chrono 
import builtins 

shapes_dir = 'ComauR3_shapes/' 
mymat = chrono.ChMaterialSurfaceNSC()
if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('HAND_e_finger-1')
body_1.SetPos(chrono.ChVectorD(-0.52964000188,0.684998394063328,1.11022302462516e-16))
body_1.SetRot(chrono.ChQuaternionD(0.707106781186546,1.43973102426035e-15,-1.03316459816659e-15,0.707106781186549))
body_1.SetMass(0.0938874355859725)
body_1.SetInertiaXX(chrono.ChVectorD(1.89819866463215e-05,4.06468272765221e-05,3.48488688224795e-05))
body_1.SetInertiaXY(chrono.ChVectorD(9.26966996175154e-06,-9.45479551365122e-06,4.9223535683921e-06))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.00724869989644222,0.0206425177285978,0.0224205323500859),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_1.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=-6.60847038467355E-16 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=-6.93889390390723E-16 
mr[0,2]=-6.60847038467355E-16; mr[1,2]=6.93889390390723E-16; mr[2,2]=-1 
body_1.GetCollisionModel().AddBox(mymat, 0.0105,0.01,0.00299999999999999,chrono.ChVectorD(-1.0000000000005E-05,0.0501,0.0279999999999992),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('Racer3_p04-1')
body_2.SetPos(chrono.ChVectorD(-0.0500000000000003,0.635,-4.16333634234434e-17))
body_2.SetRot(chrono.ChQuaternionD(-3.46944695195361e-17,-1.80411241501588e-15,1,-9.71445146547012e-17))
body_2.SetMass(2.57881604056105)
body_2.SetInertiaXX(chrono.ChVectorD(0.00508982512139297,0.00588527304221938,0.00712171226614457))
body_2.SetInertiaXY(chrono.ChVectorD(0.00171483285772537,0.000111903320303156,-0.000118834560475186))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0080293005188847,0.0195160789485397,0.00189129063974129),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_2.GetAssets().push_back(body_2_1_level) 

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('Racer3_p03-1')
body_3.SetPos(chrono.ChVectorD(-0.05,0.365,-1.21430643318377e-17))
body_3.SetRot(chrono.ChQuaternionD(0.707106781186547,-2.4532694666934e-17,0.707106781186548,2.4532694666934e-17))
body_3.SetMass(10.3728089874946)
body_3.SetInertiaXX(chrono.ChVectorD(0.176466622032337,0.0661422545401289,0.135351077582464))
body_3.SetInertiaXY(chrono.ChVectorD(-0.000470896812626748,0.00053842727264319,-0.00941817630357473))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00640563915569544,0.13656110939561,0.0103504727018846),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_3.GetAssets().push_back(body_3_1_level) 

# Collision shapes 
body_3.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=1; mr[1,1]=-1.72563323017096E-31; mr[2,1]=0 
mr[0,2]=-1.72563323017096E-31; mr[1,2]=-1; mr[2,2]=0 
body_3.GetCollisionModel().AddCylinder(mymat, 0.075,0.075,0.055,chrono.ChVectorD(-0.00500000000000001,-1.30975562169976E-32,0),mr)
body_3.GetCollisionModel().BuildModel()
body_3.SetCollide(True)

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('Racer3_p02-1')
body_4.SetPos(chrono.ChVectorD(0,0,0))
body_4.SetRot(chrono.ChQuaternionD(-2.77555756156289e-17,3.00871725494699e-18,1,-3.12897393905727e-17))
body_4.SetMass(5.77385036241004)
body_4.SetInertiaXX(chrono.ChVectorD(0.0597089893302415,0.0472381748997652,0.0354880462383778))
body_4.SetInertiaXY(chrono.ChVectorD(0.00446817551391682,-0.000103460098873769,1.35479368942586e-05))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0376094579848077,0.309755519962188,0.00462319569912938),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChObjShapeFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4_1_level = chrono.ChAssetLevel() 
body_4_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_4_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_4_1_level.GetAssets().push_back(body_4_1_shape) 
body_4.GetAssets().push_back(body_4_1_level) 

exported_items.append(body_4)



# Rigid body part
body_5= chrono.ChBodyAuxRef()
body_5.SetName('Racer3_p05-1')
body_5.SetPos(chrono.ChVectorD(-0.109,0.68499839406333,0))
body_5.SetRot(chrono.ChQuaternionD(0.707106781186548,-1.22172819441331e-15,0.707106781186547,1.22172819441331e-15))
body_5.SetMass(4.23269774636928)
body_5.SetInertiaXX(chrono.ChVectorD(0.00845668617506054,0.0295974459040003,0.0294943258300645))
body_5.SetInertiaXY(chrono.ChVectorD(1.95546080844713e-06,-0.000578717905123499,-1.89364192936429e-05))
body_5.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00215407384493712,-0.00454635604425113,-0.115249310209588),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChObjShapeFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5_1_level = chrono.ChAssetLevel() 
body_5_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_5_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_5_1_level.GetAssets().push_back(body_5_1_shape) 
body_5.GetAssets().push_back(body_5_1_level) 

# Collision shapes 
body_5.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_5.GetCollisionModel().AddCylinder(mymat, 0.0445270099166724,0.0445270099166724,0.02,chrono.ChVectorD(0.041,0,-0.24694),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=-1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=0 
body_5.GetCollisionModel().AddCylinder(mymat, 0.0445270099166724,0.0445270099166724,0.015,chrono.ChVectorD(-0.045,0,-0.24694),mr)
body_5.GetCollisionModel().BuildModel()
body_5.SetCollide(True)

exported_items.append(body_5)



# Rigid body part
body_6= chrono.ChBodyAuxRef()
body_6.SetName('Hand_base_and_p07-2')
body_6.SetPos(chrono.ChVectorD(-0.48934000094,0.684998394063328,0))
body_6.SetRot(chrono.ChQuaternionD(-1.07241690963369e-15,0.707106781186549,-0.707106781186546,-1.40047871279326e-15))
body_6.SetMass(0.38232347669012)
body_6.SetInertiaXX(chrono.ChVectorD(0.000562341169881375,0.000200642758263749,0.000560005474220585))
body_6.SetInertiaXY(chrono.ChVectorD(2.53772793696237e-07,1.18964016585412e-09,5.52715203788699e-07))
body_6.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1.48931313925549e-05,-0.0107960376660547,5.81128445806255e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChObjShapeFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6_1_level = chrono.ChAssetLevel() 
body_6_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_6_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_6_1_level.GetAssets().push_back(body_6_1_shape) 
body_6.GetAssets().push_back(body_6_1_level) 

exported_items.append(body_6)



# Rigid body part
body_7= chrono.ChBodyAuxRef()
body_7.SetName('HAND_e_finger-2')
body_7.SetPos(chrono.ChVectorD(-0.52964000188,0.684998394063328,0))
body_7.SetRot(chrono.ChQuaternionD(-6.99519950696291e-16,0.707106781186549,-0.707106781186546,-1.77337567173065e-15))
body_7.SetMass(0.0938874355859725)
body_7.SetInertiaXX(chrono.ChVectorD(1.89819866463215e-05,4.06468272765222e-05,3.48488688224795e-05))
body_7.SetInertiaXY(chrono.ChVectorD(-9.26966996175154e-06,-9.45479551365122e-06,-4.9223535683921e-06))
body_7.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.00724869989644222,0.0206425177285978,0.0224205323500859),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_7.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_7.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=-6.60847038467355E-16 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=-6.93889390390723E-16 
mr[0,2]=-6.60847038467355E-16; mr[1,2]=6.93889390390723E-16; mr[2,2]=-1 
body_7.GetCollisionModel().AddBox(mymat, 0.0105,0.01,0.00299999999999999,chrono.ChVectorD(-1.0000000000005E-05,0.0501,0.0279999999999992),mr)
body_7.GetCollisionModel().BuildModel()
body_7.SetCollide(True)

exported_items.append(body_7)



# Rigid body part
body_8= chrono.ChBodyAuxRef()
body_8.SetName('Racer3_p06-1')
body_8.SetPos(chrono.ChVectorD(-0.355939999999991,0.684998394063334,9.26992857475089e-17))
body_8.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_8.SetMass(2.10059663848927)
body_8.SetInertiaXX(chrono.ChVectorD(0.00108997735746697,0.00212849034545108,0.00206095104131987))
body_8.SetInertiaXY(chrono.ChVectorD(2.59788983804713e-07,-7.16858194423443e-05,2.74195861167327e-07))
body_8.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.0272921807259741,-5.5585435857816e-06,0.00137129480335453),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_8_1_shape = chrono.ChObjShapeFile() 
body_8_1_shape.SetFilename(shapes_dir +'body_8_1.obj') 
body_8_1_level = chrono.ChAssetLevel() 
body_8_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_8_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_8_1_level.GetAssets().push_back(body_8_1_shape) 
body_8.GetAssets().push_back(body_8_1_level) 

exported_items.append(body_8)



# Rigid body part
body_9= chrono.ChBodyAuxRef()
body_9.SetName('Racer3_p01-3')
body_9.SetPos(chrono.ChVectorD(0,0,0))
body_9.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_9.SetMass(2.0311208710522)
body_9.SetInertiaXX(chrono.ChVectorD(0.0178704565174335,0.0139707976830781,0.0215332108939413))
body_9.SetInertiaXY(chrono.ChVectorD(0.00154955689907001,-1.0360840536981e-05,3.57379111199809e-06))
body_9.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0165738007105043,0.143198831373498,6.77868185580503e-05),chrono.ChQuaternionD(1,0,0,0)))
body_9.SetBodyFixed(True)

# Visualization shape 
body_9_1_shape = chrono.ChObjShapeFile() 
body_9_1_shape.SetFilename(shapes_dir +'body_9_1.obj') 
body_9_1_level = chrono.ChAssetLevel() 
body_9_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_9_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_9_1_level.GetAssets().push_back(body_9_1_shape) 
body_9.GetAssets().push_back(body_9_1_level) 

# Collision shapes 
body_9.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=0; mr[1,0]=0; mr[2,0]=-1 
mr[0,1]=0; mr[1,1]=-1; mr[2,1]=0 
mr[0,2]=-1; mr[1,2]=0; mr[2,2]=0 
body_9.GetCollisionModel().AddCylinder(mymat, 0.118566017177982,0.118566017177982,0.09425,chrono.ChVectorD(0,0.09425,0),mr)
body_9.GetCollisionModel().BuildModel()
body_9.SetCollide(True)

exported_items.append(body_9)




# Mate constraint: Angle1 [MatePlanarAngleDim] type:6 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: Racer3_p02-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name:  , SW name: Racer3_HandE ,  SW ref.type:4 (4)


# Mate constraint: Angle2 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: Racer3_p03-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_4 , SW name: Racer3_p02-1 ,  SW ref.type:4 (4)


# Mate constraint: Angle3 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Racer3_p04-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_3 , SW name: Racer3_p03-1 ,  SW ref.type:4 (4)


# Mate constraint: Angle4 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Racer3_p04-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_5 , SW name: Racer3_p05-1 ,  SW ref.type:4 (4)


# Mate constraint: Angle5 [MatePlanarAngleDim] type:6 align:0 flip:True
#   Entity 0: C::E name: body_5 , SW name: Racer3_p05-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_8 , SW name: Racer3_p06-1 ,  SW ref.type:4 (4)


# Mate constraint: Angle6 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: Hand_base_and_p07-2 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_8 , SW name: Racer3_p06-1 ,  SW ref.type:4 (4)


# Mate constraint: Concentric1 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_9 , SW name: Racer3_p01-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Racer3_p02-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0.00211060633115506,0)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(1.12826897060512e-18,0.1875,-1.17336522714648e-17)
dB = chrono.ChVectorD(6.01743450989396e-18,1,-6.25794787811454e-17)
link_1.Initialize(body_9,body_4,False,cA,cB,dA,dB)
link_1.SetName("Concentric1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0,0.00211060633115506,0)
cB = chrono.ChVectorD(1.12826897060512e-18,0.1875,-1.17336522714648e-17)
dA = chrono.ChVectorD(0,1,0)
dB = chrono.ChVectorD(6.01743450989396e-18,1,-6.25794787811454e-17)
link_2.Initialize(body_9,body_4,False,cA,cB,dA,dB)
link_2.SetName("Concentric1")
exported_items.append(link_2)


# Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Racer3_p01-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Racer3_p02-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0.242,0)
cB = chrono.ChVectorD(-1.9438388615202e-18,0.242,-0.06125)
dA = chrono.ChVectorD(0,1,0)
dB = chrono.ChVectorD(-6.01743450989396e-18,-1,6.25794787811454e-17)
link_3.Initialize(body_9,body_4,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident1")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0.242,0)
dA = chrono.ChVectorD(0,1,0)
cB = chrono.ChVectorD(-1.9438388615202e-18,0.242,-0.06125)
dB = chrono.ChVectorD(-6.01743450989396e-18,-1,6.25794787811454e-17)
link_4.SetFlipped(True)
link_4.Initialize(body_9,body_4,False,cA,cB,dA,dB)
link_4.SetName("Coincident1")
exported_items.append(link_4)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: Racer3_p02-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Racer3_p03-1 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.05,0.365,-0.061)
dA = chrono.ChVectorD(6.93889390390723e-17,-6.25794787811455e-17,-1)
cB = chrono.ChVectorD(-0.05,0.365,0.0809)
dB = chrono.ChVectorD(1.11022302462516e-16,1.72563323017096e-31,1)
link_5.SetFlipped(True)
link_5.Initialize(body_4,body_3,False,cA,cB,dA,dB)
link_5.SetName("Concentric2")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.05,0.365,-0.061)
cB = chrono.ChVectorD(-0.05,0.365,0.0809)
dA = chrono.ChVectorD(6.93889390390723e-17,-6.25794787811455e-17,-1)
dB = chrono.ChVectorD(1.11022302462516e-16,1.72563323017096e-31,1)
link_6.Initialize(body_4,body_3,False,cA,cB,dA,dB)
link_6.SetName("Concentric2")
exported_items.append(link_6)


# Mate constraint: Coincident2 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_4 , SW name: Racer3_p02-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_3 , SW name: Racer3_p03-1 ,  SW ref.type:4 (4)

link_7 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0,0,0)
cB = chrono.ChVectorD(-0.05,0.365,-1.21430643318377e-17)
dA = chrono.ChVectorD(-5.55111512312578e-17,-6.25794787811454e-17,-1)
dB = chrono.ChVectorD(-1.11022302462516e-16,0,-1)
link_7.Initialize(body_4,body_3,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("Coincident2")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0,0,0)
dA = chrono.ChVectorD(-5.55111512312578e-17,-6.25794787811454e-17,-1)
cB = chrono.ChVectorD(-0.05,0.365,-1.21430643318377e-17)
dB = chrono.ChVectorD(-1.11022302462516e-16,0,-1)
link_8.Initialize(body_4,body_3,False,cA,cB,dA,dB)
link_8.SetName("Coincident2")
exported_items.append(link_8)


# Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: Racer3_p03-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Racer3_p04-1 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0500000000000001,0.635,-0.142010463479011)
dA = chrono.ChVectorD(-1.11022302462516e-16,3.53527706781886e-16,-1)
cB = chrono.ChVectorD(-0.0500000000000003,0.635,3.40005801291454e-16)
dB = chrono.ChVectorD(1.3877787807815e-16,-1.02138517399596e-15,1)
link_9.SetFlipped(True)
link_9.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_9.SetName("Concentric3")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.0500000000000001,0.635,-0.142010463479011)
cB = chrono.ChVectorD(-0.0500000000000003,0.635,3.40005801291454e-16)
dA = chrono.ChVectorD(-1.11022302462516e-16,3.53527706781886e-16,-1)
dB = chrono.ChVectorD(1.3877787807815e-16,-1.02138517399596e-15,1)
link_10.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_10.SetName("Concentric3")
exported_items.append(link_10)


# Mate constraint: Coincident3 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_3 , SW name: Racer3_p03-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_2 , SW name: Racer3_p04-1 ,  SW ref.type:4 (4)

link_11 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.05,0.365,-1.21430643318377e-17)
cB = chrono.ChVectorD(-0.0500000000000003,0.635,-4.16333634234434e-17)
dA = chrono.ChVectorD(-1.11022302462516e-16,0,-1)
dB = chrono.ChVectorD(-1.38777878078145e-16,-2.77555756156289e-16,-1)
link_11.Initialize(body_3,body_2,False,cA,cB,dB)
link_11.SetDistance(0)
link_11.SetName("Coincident3")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.05,0.365,-1.21430643318377e-17)
dA = chrono.ChVectorD(-1.11022302462516e-16,0,-1)
cB = chrono.ChVectorD(-0.0500000000000003,0.635,-4.16333634234434e-17)
dB = chrono.ChVectorD(-1.38777878078145e-16,-2.77555756156289e-16,-1)
link_12.Initialize(body_3,body_2,False,cA,cB,dA,dB)
link_12.SetName("Coincident3")
exported_items.append(link_12)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Racer3_p04-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: Racer3_p05-1 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0588114984862639,0.68499839406333,-4.71834532188352e-17)
dA = chrono.ChVectorD(1,3.53883589099268e-15,6.20919814070444e-31)
cB = chrono.ChVectorD(-0.109,0.68499839406333,0)
dB = chrono.ChVectorD(-1,-3.4555691641458e-15,1.11022302462516e-16)
link_13.SetFlipped(True)
link_13.Initialize(body_2,body_5,False,cA,cB,dA,dB)
link_13.SetName("Concentric4")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.0588114984862639,0.68499839406333,-4.71834532188352e-17)
cB = chrono.ChVectorD(-0.109,0.68499839406333,0)
dA = chrono.ChVectorD(1,3.53883589099268e-15,6.20919814070444e-31)
dB = chrono.ChVectorD(-1,-3.4555691641458e-15,1.11022302462516e-16)
link_14.Initialize(body_2,body_5,False,cA,cB,dA,dB)
link_14.SetName("Concentric4")
exported_items.append(link_14)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Racer3_p04-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: Racer3_p05-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.117800000000001,0.70749839406333,-4.9682302057189e-17)
cB = chrono.ChVectorD(-0.1178,0.68499839406333,-0.0225)
dA = chrono.ChVectorD(-1,-3.53883589099268e-15,-6.20919814070444e-31)
dB = chrono.ChVectorD(1,3.4555691641458e-15,-1.11022302462516e-16)
link_15.Initialize(body_2,body_5,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident4")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.117800000000001,0.70749839406333,-4.9682302057189e-17)
dA = chrono.ChVectorD(-1,-3.53883589099268e-15,-6.20919814070444e-31)
cB = chrono.ChVectorD(-0.1178,0.68499839406333,-0.0225)
dB = chrono.ChVectorD(1,3.4555691641458e-15,-1.11022302462516e-16)
link_16.SetFlipped(True)
link_16.Initialize(body_2,body_5,False,cA,cB,dA,dB)
link_16.SetName("Coincident4")
exported_items.append(link_16)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_5 , SW name: Racer3_p05-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Racer3_p06-1 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.35594,0.684998394063329,-0.021)
dA = chrono.ChVectorD(-2.77555756156289e-17,-5.55111512312578e-17,-1)
cB = chrono.ChVectorD(-0.355939999999991,0.684998394063334,-0.0381038482126925)
dB = chrono.ChVectorD(-1.22460635382238e-16,0,-1)
link_17.Initialize(body_5,body_8,False,cA,cB,dA,dB)
link_17.SetName("Concentric5")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateGeneric()
link_18.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.35594,0.684998394063329,-0.021)
cB = chrono.ChVectorD(-0.355939999999991,0.684998394063334,-0.0381038482126925)
dA = chrono.ChVectorD(-2.77555756156289e-17,-5.55111512312578e-17,-1)
dB = chrono.ChVectorD(-1.22460635382238e-16,0,-1)
link_18.Initialize(body_5,body_8,False,cA,cB,dA,dB)
link_18.SetName("Concentric5")
exported_items.append(link_18)


# Mate constraint: Coincident5 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: Racer3_p05-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Racer3_p06-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.35594,0.684998394063329,-0.021)
cB = chrono.ChVectorD(-0.380939999999991,0.684998394063334,-0.0209999999999999)
dA = chrono.ChVectorD(2.77555756156289e-17,5.55111512312578e-17,1)
dB = chrono.ChVectorD(-1.22460635382238e-16,0,-1)
link_19.Initialize(body_5,body_8,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident5")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.35594,0.684998394063329,-0.021)
dA = chrono.ChVectorD(2.77555756156289e-17,5.55111512312578e-17,1)
cB = chrono.ChVectorD(-0.380939999999991,0.684998394063334,-0.0209999999999999)
dB = chrono.ChVectorD(-1.22460635382238e-16,0,-1)
link_20.SetFlipped(True)
link_20.Initialize(body_5,body_8,False,cA,cB,dA,dB)
link_20.SetName("Coincident5")
exported_items.append(link_20)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_8 , SW name: Racer3_p06-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: Hand_base_and_p07-2 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.431439999999991,0.684998394063334,9.26992857475089e-17)
dA = chrono.ChVectorD(1,0,0)
cB = chrono.ChVectorD(-0.44394,0.684998394063328,-2.10633055262596e-17)
dB = chrono.ChVectorD(1,3.49717290997721e-15,2.81803457385138e-17)
link_21.Initialize(body_8,body_6,False,cA,cB,dA,dB)
link_21.SetName("Concentric6")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateGeneric()
link_22.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.431439999999991,0.684998394063334,9.26992857475089e-17)
cB = chrono.ChVectorD(-0.44394,0.684998394063328,-2.10633055262596e-17)
dA = chrono.ChVectorD(1,0,0)
dB = chrono.ChVectorD(1,3.49717290997721e-15,2.81803457385138e-17)
link_22.Initialize(body_8,body_6,False,cA,cB,dA,dB)
link_22.SetName("Concentric6")
exported_items.append(link_22)


# Mate constraint: Coincident6 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_8 , SW name: Racer3_p06-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: Hand_base_and_p07-2 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.426939999999991,0.684998394063334,9.26992857475089e-17)
cB = chrono.ChVectorD(-0.426940000000001,0.700498394063328,6.36005895978425e-17)
dA = chrono.ChVectorD(-1,0,0)
dB = chrono.ChVectorD(1,3.49717290997721e-15,2.81803457385168e-17)
link_23.Initialize(body_8,body_6,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident6")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.426939999999991,0.684998394063334,9.26992857475089e-17)
dA = chrono.ChVectorD(-1,0,0)
cB = chrono.ChVectorD(-0.426940000000001,0.700498394063328,6.36005895978425e-17)
dB = chrono.ChVectorD(1,3.49717290997721e-15,2.81803457385168e-17)
link_24.SetFlipped(True)
link_24.Initialize(body_8,body_6,False,cA,cB,dA,dB)
link_24.SetName("Coincident6")
exported_items.append(link_24)


# Mate constraint: Coincidente3 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: HAND_e_finger-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_6 , SW name: Hand_base_and_p07-2 ,  SW ref.type:4 (4)

link_25 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.52964000188,0.684998394063328,1.11022302462516e-16)
cB = chrono.ChVectorD(-0.48934000094,0.684998394063328,0)
dA = chrono.ChVectorD(-4.29660276414192e-15,1,3.49720252756924e-15)
dB = chrono.ChVectorD(4.36599170318099e-15,-1,-3.49720252756924e-15)
link_25.Initialize(body_1,body_6,False,cA,cB,dB)
link_25.SetDistance(0)
link_25.SetName("Coincidente3")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.52964000188,0.684998394063328,1.11022302462516e-16)
dA = chrono.ChVectorD(-4.29660276414192e-15,1,3.49720252756924e-15)
cB = chrono.ChVectorD(-0.48934000094,0.684998394063328,0)
dB = chrono.ChVectorD(4.36599170318099e-15,-1,-3.49720252756924e-15)
link_26.SetFlipped(True)
link_26.Initialize(body_1,body_6,False,cA,cB,dA,dB)
link_26.SetName("Coincidente3")
exported_items.append(link_26)


# Mate constraint: Coincidente5 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: Hand_base_and_p07-2 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_7 , SW name: HAND_e_finger-2 ,  SW ref.type:4 (4)

link_27 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.48934000094,0.684998394063328,0)
cB = chrono.ChVectorD(-0.52964000188,0.684998394063328,0)
dA = chrono.ChVectorD(4.36599170318099e-15,-1,-3.49720252756924e-15)
dB = chrono.ChVectorD(4.42150285441224e-15,-1,-3.49720252756924e-15)
link_27.Initialize(body_6,body_7,False,cA,cB,dB)
link_27.SetDistance(0)
link_27.SetName("Coincidente5")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.48934000094,0.684998394063328,0)
dA = chrono.ChVectorD(4.36599170318099e-15,-1,-3.49720252756924e-15)
cB = chrono.ChVectorD(-0.52964000188,0.684998394063328,0)
dB = chrono.ChVectorD(4.42150285441224e-15,-1,-3.49720252756924e-15)
link_28.Initialize(body_6,body_7,False,cA,cB,dA,dB)
link_28.SetName("Coincidente5")
exported_items.append(link_28)


# Mate constraint: Coincidente6 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: Hand_base_and_p07-2 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_7 , SW name: HAND_e_finger-2 ,  SW ref.type:4 (4)

link_29 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.48934000094,0.684998394063328,0)
cB = chrono.ChVectorD(-0.52964000188,0.684998394063328,0)
dA = chrono.ChVectorD(-4.63949451324841e-16,3.49720252756924e-15,-1)
dB = chrono.ChVectorD(-1.51866132471875e-15,3.49720252756924e-15,-1)
link_29.Initialize(body_6,body_7,False,cA,cB,dB)
link_29.SetDistance(0)
link_29.SetName("Coincidente6")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.48934000094,0.684998394063328,0)
dA = chrono.ChVectorD(-4.63949451324841e-16,3.49720252756924e-15,-1)
cB = chrono.ChVectorD(-0.52964000188,0.684998394063328,0)
dB = chrono.ChVectorD(-1.51866132471875e-15,3.49720252756924e-15,-1)
link_30.Initialize(body_6,body_7,False,cA,cB,dA,dB)
link_30.SetName("Coincidente6")
exported_items.append(link_30)


# Mate constraint: Coincidente8 [MateCoincident] type:0 align:0 flip:False
#   Entity 0: C::E name: body_1 , SW name: HAND_e_finger-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: HAND_e_finger-2 ,  SW ref.type:2 (2)

link_31 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.589740001880001,0.684988394063327,0.0190139999999994)
cB = chrono.ChVectorD(-0.58974000188,0.685008394063328,-0.0190139999999992)
dA = chrono.ChVectorD(-1,-4.6152482751252e-15,7.81268335880247e-16)
dB = chrono.ChVectorD(-1,-4.10285734342897e-15,1.31236474262583e-15)
link_31.Initialize(body_1,body_7,False,cA,cB,dB)
link_31.SetDistance(0)
link_31.SetName("Coincidente8")
exported_items.append(link_31)

link_32 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.589740001880001,0.684988394063327,0.0190139999999994)
dA = chrono.ChVectorD(-1,-4.6152482751252e-15,7.81268335880247e-16)
cB = chrono.ChVectorD(-0.58974000188,0.685008394063328,-0.0190139999999992)
dB = chrono.ChVectorD(-1,-4.10285734342897e-15,1.31236474262583e-15)
link_32.Initialize(body_1,body_7,False,cA,cB,dA,dB)
link_32.SetName("Coincidente8")
exported_items.append(link_32)


# Mate constraint: Coincidente12 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: Hand_base_and_p07-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: HAND_e_finger-2 ,  SW ref.type:2 (2)

link_33 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.53874000188,0.692048394063328,0.0375)
cB = chrono.ChVectorD(-0.538740001880001,0.692148394063328,-0.0365139999999992)
dA = chrono.ChVectorD(1,4.36599170318098e-15,-4.63949451324826e-16)
dB = chrono.ChVectorD(-1,-4.10311231612994e-15,1.62118497111516e-15)
link_33.Initialize(body_6,body_7,False,cA,cB,dB)
link_33.SetDistance(0)
link_33.SetName("Coincidente12")
exported_items.append(link_33)

link_34 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.53874000188,0.692048394063328,0.0375)
dA = chrono.ChVectorD(1,4.36599170318098e-15,-4.63949451324826e-16)
cB = chrono.ChVectorD(-0.538740001880001,0.692148394063328,-0.0365139999999992)
dB = chrono.ChVectorD(-1,-4.10311231612994e-15,1.62118497111516e-15)
link_34.SetFlipped(True)
link_34.Initialize(body_6,body_7,False,cA,cB,dA,dB)
link_34.SetName("Coincidente12")
exported_items.append(link_34)


# Mate constraint: Coincidente13 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: HAND_e_finger-1 ,  SW ref.type:4 (4)
#   Entity 1: C::E name: body_6 , SW name: Hand_base_and_p07-2 ,  SW ref.type:4 (4)

link_35 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.52964000188,0.684998394063328,1.11022302462516e-16)
cB = chrono.ChVectorD(-0.48934000094,0.684998394063328,0)
dA = chrono.ChVectorD(5.74971753787358e-16,-3.49720252756924e-15,1)
dB = chrono.ChVectorD(-4.63949451324841e-16,3.49720252756924e-15,-1)
link_35.Initialize(body_1,body_6,False,cA,cB,dB)
link_35.SetDistance(0)
link_35.SetName("Coincidente13")
exported_items.append(link_35)

link_36 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.52964000188,0.684998394063328,1.11022302462516e-16)
dA = chrono.ChVectorD(5.74971753787358e-16,-3.49720252756924e-15,1)
cB = chrono.ChVectorD(-0.48934000094,0.684998394063328,0)
dB = chrono.ChVectorD(-4.63949451324841e-16,3.49720252756924e-15,-1)
link_36.SetFlipped(True)
link_36.Initialize(body_1,body_6,False,cA,cB,dA,dB)
link_36.SetName("Coincidente13")
exported_items.append(link_36)

