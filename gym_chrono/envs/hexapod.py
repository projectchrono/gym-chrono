# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: D:\files\CAD\Hexapod\Hexapod.SLDASM


import pychrono as chrono 
import builtins 

shapes_dir = 'hexapod_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)
mymat = chrono.ChMaterialSurfaceNSC()
# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('Tibia-1')
body_1.SetPos(chrono.ChVectorD(-0.151653435008421,-0.00132652747527713,0.21284754341261))
body_1.SetRot(chrono.ChQuaternionD(0.805847475772168,-0.187160021072023,0.333792953668956,-0.451844261206113))
body_1.SetMass(0.0749832765369386)
body_1.SetInertiaXX(chrono.ChVectorD(4.58262640873382e-05,0.000108433975599404,8.9530940983583e-05))
body_1.SetInertiaXY(chrono.ChVectorD(-2.69334937812053e-06,-3.45986868931097e-05,-1.36555187966809e-06))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0159616525902916,-0.0203081631545103,0.000240966541478178),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0784186804226129,-0.0590194226200942,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(3.53865624873292E-16,-0.489072704076376,0.872243022401109,-5.17603351857777E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_1.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0643520251386334,-0.0666436740011606,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(3.53865624873292E-16,-0.489072704076376,0.872243022401109,-5.17603351857777E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_1.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-1.7255368887123E-05)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_1.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-0.0184172553688871)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_1.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0119591321615523,0.0211063259988394,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_1.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0129591321615523,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_1.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00804086783844774,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_1.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-0.00201725536888712)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.77555756156289E-17,7.70371977754894E-34,-2.77555756156289E-17,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_1.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00249086783844774,-0.00258367400116064,-0.00976725536888715)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,-1.96261557335472E-17,1.96261557335472E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_1.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00750913216155227,-0.00258367400116064,-0.00976725536888713)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,-1.96261557335472E-17,1.96261557335472E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_1.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00704086783844773,0.0211063259988394,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_1.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,-0.00764367400116064,-0.0160172553688871)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,-0.707106781186547,0.707106781186548,1.96261557335472E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_1.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155225,-0.0076436740011607,0.0184827446311129)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(7.85046229341887E-17,-0.707106781186547,0.707106781186548,-5.88784672006416E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_1.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0106052050870878,-0.0516436740011606,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(4.41641787476618E-16,-0.236184670402587,0.97170818740341,-4.06800463549082E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_1.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_1.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_1.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.00245913216155228,-0.00664367400116065,0.00148274463111287),mr)
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)



# Rigid body part
body_2= chrono.ChBodyAuxRef()
body_2.SetName('Body-1')
body_2.SetPos(chrono.ChVectorD(0,0,0))
body_2.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_2.SetMass(0.523119764657616)
body_2.SetInertiaXX(chrono.ChVectorD(0.00343395197207475,0.00488069992185034,0.00165362362984408))
body_2.SetInertiaXY(chrono.ChVectorD(-1.41697255304125e-07,2.95285471512551e-07,-1.53816834697879e-05))
body_2.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000357126198687995,0.0192584708003123,-0.0016327733630494),chrono.ChQuaternionD(1,0,0,0)))
body_2.SetBodyFixed(True)

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(-1.73472347597681E-18,0.0185,0.09675)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_2.GetAssets().push_back(body_2_1_level) 

# Visualization shape 
body_2_1_shape = chrono.ChObjShapeFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2_1_level = chrono.ChAssetLevel() 
body_2_1_level.GetFrame().SetPos(chrono.ChVectorD(-1.73472347597681E-18,0.0185,-0.09925)) 
body_2_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_1_level.GetAssets().push_back(body_2_1_shape) 
body_2.GetAssets().push_back(body_2_1_level) 

# Visualization shape 
body_2_3_shape = chrono.ChObjShapeFile() 
body_2_3_shape.SetFilename(shapes_dir +'body_2_3.obj') 
body_2_3_level = chrono.ChAssetLevel() 
body_2_3_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_2_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_3_level.GetAssets().push_back(body_2_3_shape) 
body_2.GetAssets().push_back(body_2_3_level) 

# Visualization shape 
body_2_4_shape = chrono.ChObjShapeFile() 
body_2_4_shape.SetFilename(shapes_dir +'body_2_4.obj') 
body_2_4_level = chrono.ChAssetLevel() 
body_2_4_level.GetFrame().SetPos(chrono.ChVectorD(2.94902990916057E-17,0.0345,2.77555756156289E-17)) 
body_2_4_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_2_4_level.GetAssets().push_back(body_2_4_shape) 
body_2.GetAssets().push_back(body_2_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.0609100775541958,0.0185,0.121910077554196)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.270598050073099,-0.270598050073098,0.653281482438188,0.653281482438188)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_2.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.0609100775541958,0.000100000000000024,0.121910077554196)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,2.19284466576703E-16,-0.38268343236509,-5.29399533227226E-16)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_2.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0736379996155536,0.033,0.121202970773009)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,2.19284466576703E-16,-0.38268343236509,-5.29399533227226E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0450001749774984,0.033,0.091150932572581)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,2.19284466576703E-16,-0.38268343236509,-5.29399533227226E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0301509325725809,0.033,0.106000174977499)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,2.19284466576703E-16,-0.38268343236509,-5.29399533227226E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.0609100775541958,0.0165,0.121910077554196)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0.653281482438188,-0.653281482438188,-0.270598050073098,-0.270598050073099)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_2.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.0466689469810987,0.00875000000000002,0.114669304114846)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(5.29399533227226E-16,-0.38268343236509,-2.19284466576703E-16,0.923879532511287)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.0537400147929642,0.00875000000000003,0.10759823630298)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(5.29399533227226E-16,-0.38268343236509,-2.19284466576703E-16,0.923879532511287)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0602029707730092,0.033,0.134637999615554)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,2.19284466576703E-16,-0.38268343236509,-5.29399533227226E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(-0.0125,0.0035,0.0875)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.641265573006503,-0.641265573006503,-0.297957152752944,-0.297957152752944)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_2.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(-0.0125,0.0035,-0.0875)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.641265573006503,-0.641265573006503,-0.297957152752944,-0.297957152752944)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_2.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0125,0.0035,-0.0875)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.641265573006503,-0.641265573006503,-0.297957152752944,-0.297957152752944)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_2.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.1035,0.0185000000000002,2.08166817117217E-17)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.5,-0.5,0.5,0.5)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_2.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.1035,0.000100000000000162,9.3481602110062E-18)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-1.19653972408696E-16,-0.707106781186547,-5.60386000851068E-16)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_2.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.112,0.0330000000000002,-0.00949999999999997)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-1.19653972408696E-16,-0.707106781186547,-5.60386000851068E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0705,0.0330000000000001,-0.0105)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-1.19653972408696E-16,-0.707106781186547,-5.60386000851068E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0705,0.0330000000000001,0.0105)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-1.19653972408696E-16,-0.707106781186547,-5.60386000851068E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.1035,0.0165000000000002,1.95701032877309E-17)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.5,0.5,0.5,0.5)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_2.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.08831,0.00875000000000014,0.00495000000000001)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(5.60386000851068E-16,-0.707106781186547,1.19653972408696E-16,0.707106781186548)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.08831,0.00875000000000015,-0.00504999999999999)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(5.60386000851068E-16,-0.707106781186547,1.19653972408696E-16,0.707106781186548)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.112,0.0330000000000002,0.00950000000000003)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-1.19653972408696E-16,-0.707106781186547,-5.60386000851068E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.0609100775541958,0.0165000000000001,0.121910077554196)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0.653281482438189,-0.653281482438188,0.270598050073099,0.270598050073099)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_2.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0602029707730093,0.0330000000000001,0.134637999615554)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,5.71377890954441E-16,0.38268343236509,4.33226129346829E-17)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0450001749774985,0.0330000000000001,0.091150932572581)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,5.71377890954441E-16,0.38268343236509,4.33226129346829E-17)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0736379996155537,0.0330000000000001,0.121202970773009)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,5.71377890954441E-16,0.38268343236509,4.33226129346829E-17)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.030150932572581,0.0330000000000001,0.106000174977499)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,5.71377890954441E-16,0.38268343236509,4.33226129346829E-17)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.0465982363029801,0.00875000000000011,0.114740014792964)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(-4.33226129346829E-17,0.38268343236509,-5.71377890954441E-16,0.923879532511287)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.0536693041148456,0.00875000000000012,0.107668946981099)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(-4.33226129346829E-17,0.38268343236509,-5.71377890954441E-16,0.923879532511287)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.0609100775541958,0.0001000000000001,0.121910077554196)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(0.923879532511287,5.71377890954441E-16,0.38268343236509,4.33226129346829E-17)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_2.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.0609100775541958,0.0185000000000001,0.121910077554196)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.270598050073099,0.270598050073099,0.653281482438188,0.653281482438189)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_2.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0125,0.00349999999999999,0.0875)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.641265573006503,-0.641265573006503,-0.297957152752944,-0.297957152752944)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_2.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.0609100775541958,0.0185000000000002,-0.121910077554196)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.653281482438189,-0.653281482438188,0.270598050073098,0.270598050073099)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_2.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.0609100775541958,0.000100000000000169,-0.121910077554196)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.38268343236509,-3.94898850215952E-16,0.923879532511287,4.15216137886225E-16)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_2.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0602029707730092,0.0330000000000002,-0.134637999615554)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.38268343236509,-3.94898850215952E-16,0.923879532511287,4.15216137886225E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.030150932572581,0.0330000000000002,-0.106000174977498)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.38268343236509,-3.94898850215952E-16,0.923879532511287,4.15216137886225E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0450001749774984,0.0330000000000001,-0.091150932572581)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.38268343236509,-3.94898850215952E-16,0.923879532511287,4.15216137886225E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.0609100775541958,0.0165000000000002,-0.121910077554196)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.270598050073099,0.270598050073098,0.653281482438188,0.653281482438189)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_2.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.0536693041148455,0.00875000000000016,-0.107668946981099)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(-4.15216137886225E-16,0.923879532511287,3.94898850215952E-16,-0.38268343236509)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.04659823630298,0.00875000000000017,-0.114740014792964)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(-4.15216137886225E-16,0.923879532511287,3.94898850215952E-16,-0.38268343236509)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0736379996155536,0.0330000000000002,-0.121202970773009)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.38268343236509,-3.94898850215952E-16,0.923879532511287,4.15216137886225E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.0609100775541958,0.0165000000000001,-0.121910077554196)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0.270598050073099,-0.270598050073099,0.653281482438188,0.653281482438189)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_2.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0736379996155536,0.0330000000000002,-0.121202970773009)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.38268343236509,-1.89004580419923E-17,0.923879532511287,5.72706133849506E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.030150932572581,0.0330000000000001,-0.106000174977498)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.38268343236509,-1.89004580419923E-17,0.923879532511287,5.72706133849506E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0602029707730092,0.0330000000000002,-0.134637999615554)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.38268343236509,-1.89004580419923E-17,0.923879532511287,5.72706133849506E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0450001749774984,0.0330000000000001,-0.091150932572581)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.38268343236509,-1.89004580419923E-17,0.923879532511287,5.72706133849506E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.0537400147929642,0.00875000000000013,-0.10759823630298)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(-5.72706133849506E-16,0.923879532511287,1.89004580419923E-17,0.38268343236509)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.0466689469810987,0.00875000000000013,-0.114669304114846)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(-5.72706133849506E-16,0.923879532511287,1.89004580419923E-17,0.38268343236509)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.0609100775541958,0.000100000000000149,-0.121910077554196)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(0.38268343236509,-1.89004580419923E-17,0.923879532511287,5.72706133849506E-16)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_2.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.0609100775541958,0.0185000000000001,-0.121910077554196)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.653281482438189,-0.653281482438188,-0.270598050073099,-0.270598050073099)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_2.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.1035,0.0165,4.39254351258577E-17)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0.5,-0.5,0.5,0.5)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_2.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.112,0.033,0.00950000000000002)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-4.05184860935743E-16,0.707106781186548,-4.05184860935742E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0705,0.033,-0.0105)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-4.05184860935743E-16,0.707106781186548,-4.05184860935742E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.112,0.033,-0.00949999999999998)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-4.05184860935743E-16,0.707106781186548,-4.05184860935742E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0705,0.033,0.0105)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-4.05184860935743E-16,0.707106781186548,-4.05184860935742E-16)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_2.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.08831,0.00875000000000001,0.00505000000000005)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(4.05184860935742E-16,0.707106781186548,4.05184860935743E-16,0.707106781186547)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.08831,0.00875,-0.00494999999999995)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(4.05184860935742E-16,0.707106781186548,4.05184860935743E-16,0.707106781186547)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_2.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.1035,0.00010000000000001,6.27204230856553E-17)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-4.05184860935743E-16,0.707106781186548,-4.05184860935742E-16)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_2.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.1035,0.0185,4.16333634234434E-17)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.5,0.5,0.5,0.5)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_2.GetAssets().push_back(body_1_3_level) 

# Collision shapes 
body_2.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=0; mr[2,1]=1 
mr[0,2]=0; mr[1,2]=-1; mr[2,2]=0 
body_2.GetCollisionModel().AddBox(mymat, 0.043,0.076,0.0185,chrono.ChVectorD(3.46944695195361E-17,0.0185,4.16333634234434E-17),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.707106781186547; mr[1,0]=-3.2447132549486E-32; mr[2,0]=0.707106781186547 
mr[0,1]=-0.707106781186547; mr[1,1]=1.14603585120717E-15; mr[2,1]=-0.707106781186548 
mr[0,2]=-8.10369721871485E-16; mr[1,2]=-1; mr[2,2]=-8.10369721871485E-16 
body_2.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.0472982720163547,0.02,0.108298272016355),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-1; mr[1,0]=9.61721753139795E-16; mr[2,0]=2.35742161722355E-16 
mr[0,1]=-5.55111512312578E-17; mr[1,1]=6.23289211995407E-16; mr[2,1]=-1 
mr[0,2]=-9.61721753139795E-16; mr[1,2]=-1; mr[2,2]=-6.23289211995407E-16 
body_2.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.08425,0.0200000000000001,1.33785674537557E-17),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.707106781186548; mr[1,0]=-1.02261098513081E-15; mr[2,0]=0.707106781186547 
mr[0,1]=-0.707106781186547; mr[1,1]=-5.17363455746468E-16; mr[2,1]=0.707106781186548 
mr[0,2]=-3.57263954205419E-16; mr[1,2]=-1; mr[2,2]=-1.08892636999829E-15 
body_2.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.0472982720163548,0.0200000000000001,0.108298272016355),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=-0.707106781186548; mr[1,0]=4.64976887847598E-16; mr[2,0]=-0.707106781186547 
mr[0,1]=0.707106781186547; mr[1,1]=1.04747060389287E-15; mr[2,1]=-0.707106781186548 
mr[0,2]=4.11885256614163E-16; mr[1,2]=-1; mr[2,2]=-1.06946187759827E-15 
body_2.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.0472982720163547,0.0200000000000002,-0.108298272016355),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=0.707106781186548; mr[1,0]=1.07268873472802E-15; mr[2,0]=-0.707106781186547 
mr[0,1]=0.707106781186547; mr[1,1]=-4.03406805395969E-16; mr[2,1]=0.707106781186548 
mr[0,2]=4.7325379075631E-16; mr[1,2]=-1; mr[2,2]=-1.04375716610089E-15 
body_2.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.0472982720163548,0.0200000000000001,-0.108298272016355),mr)
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=-3.2447132549486E-32; mr[2,0]=-2.91253312953612E-16 
mr[0,1]=1.11022302462516E-16; mr[1,1]=1.14603585120717E-15; mr[2,1]=1 
mr[0,2]=3.01339605878215E-31; mr[1,2]=-1; mr[2,2]=1.14603585120717E-15 
body_2.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.08425,0.02,4.43417560598007E-17),mr)
body_2.GetCollisionModel().BuildModel()
body_2.SetCollide(True)

exported_items.append(body_2)



# Rigid body part
body_3= chrono.ChBodyAuxRef()
body_3.SetName('Hip-1')
body_3.SetPos(chrono.ChVectorD(-0.0976796301758963,0.01774,0.158679630175896))
body_3.SetRot(chrono.ChQuaternionD(0.38268343236509,3.05148042600362e-17,0.923879532511287,6.33465334327671e-18))
body_3.SetMass(0.0187191370104082)
body_3.SetInertiaXX(chrono.ChVectorD(8.0035953017937e-06,9.62220940012942e-06,8.94046148663445e-06))
body_3.SetInertiaXY(chrono.ChVectorD(3.43276308382499e-07,-1.1501810066999e-06,3.18739690085474e-07))
body_3.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000962714713021326,0.000962714691254139,0.0260087993424653),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-6.93889390390723E-18,0.01961,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1.44444745829042E-34,-2.77555756156289E-17,1,-5.20417042793038E-18)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_3.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-6.93889390390723E-18,0.01916,0.052)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.5,0.5,-0.5,-0.5)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_3.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(6.93889390390723E-18,-0.021,0.052)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_3.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(6.93889390390723E-18,-3.46944695195361E-18,0.052)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(3.67990420004007E-18,0.707106781186548,-0.707106781186547,3.67990420004007E-18)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_3.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(6.93889390390723E-18,-0.0184,0.052)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.63677823941142E-32,-0.248882088995415,-3.89331934729213E-32,0.968533791758077)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_3.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_3.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(6.93889390390723E-18,-0.0235,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.20568030643786E-32,-0.285333923385196,-1.20568030643786E-33,0.958428167452111)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_3.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-0.01961,0,5.55111512312578E-17)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,5.88784672006416E-17,-5.88784672006416E-17,0.707106781186547)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_3.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-0.01916,0,2.77555756156289E-17)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,7.85046229341887E-17,-0.707106781186547,-7.85046229341887E-17)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_3.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.021,-6.93889390390723E-18,0)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,2.83262329743421E-32,-2.83262329743421E-32,0.707106781186547)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_3.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0184,-6.93889390390723E-18,2.77555756156289E-17)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,2.83262329743421E-32,2.83262329743421E-32,-0.707106781186547)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_3.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0235,-6.93889390390723E-18,2.77555756156289E-17)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.677711056485583,0.201761552128235,0.201761552128235,-0.677711056485583)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_3.GetAssets().push_back(body_3_1_level) 

exported_items.append(body_3)



# Rigid body part
body_4= chrono.ChBodyAuxRef()
body_4.SetName('Femur-1')
body_4.SetPos(chrono.ChVectorD(-0.100459944330662,0.0174633539086121,0.15802039141082))
body_4.SetRot(chrono.ChQuaternionD(0.653281482438188,-0.270598050073098,0.270598050073098,-0.653281482438188))
body_4.SetMass(0.0560807503629222)
body_4.SetInertiaXX(chrono.ChVectorD(1.23274187765565e-05,3.00016445513907e-05,2.9557502965817e-05))
body_4.SetInertiaXY(chrono.ChVectorD(-1.14197057182094e-06,-2.22499641184028e-06,2.1026864540246e-06))
body_4.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00073337382491561,-0.0207493752198647,0.00315852037890482),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342174,-0.0210078688061293)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.138086909818538,0.138086909818538,-0.693492613757902,0.693492613757902)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_4.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0159078688061293)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.138086909818538,0.138086909818538,-0.693492613757902,0.693492613757902)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_4.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0221021311938707)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.001104854345604,-0.001104854345604,0.707105918018563,0.707105918018563)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_4.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342174,0.0216521311938707)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.706001063672959,1.31301906636293E-17,-6.51385005637138E-18,0.708210772364167)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_4.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0185078688061293)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,0.707106781186547,1.4236994369266E-17,-1.90158695744405E-17)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_4.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387937,0.00149982679145716,0.00249213119387072)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_4.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.00027664609138793,0.00149982679145716,-0.0159078688061293)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.4236994369266E-17,-2.51429808260407E-17,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_4.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.00922335390861206,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.4236994369266E-17,-2.51429808260407E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_4.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0102233539086121,-0.0315001732085429,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.4236994369266E-17,-2.51429808260407E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_4.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0107766460913879,-0.0315001732085429,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.4236994369266E-17,-2.51429808260407E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_4.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387934,0.00149982679145716,0.000492131193870732)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0,0,0,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_4.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00522664609138794,-0.0136901732085429,-0.00725786880612928)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,2.51429808260407E-17,-1.4236994369266E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_4.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00477335390861207,-0.0136901732085429,-0.00725786880612928)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,2.51429808260407E-17,-1.4236994369266E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_4.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00977664609138794,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.4236994369266E-17,-2.51429808260407E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_4.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_4_15_shape = chrono.ChObjShapeFile() 
body_4_15_shape.SetFilename(shapes_dir +'body_4_15.obj') 
body_4_15_level = chrono.ChAssetLevel() 
body_4_15_level.GetFrame().SetPos(chrono.ChVectorD(-0.000259497674868787,-0.0399751463835917,0.00249213119387071)) 
body_4_15_level.GetFrame().SetRot(chrono.ChQuaternionD(0.99999877929613,0,6.32843505347574E-20,0.00156249999999996)) 
body_4_15_level.GetAssets().push_back(body_4_15_shape) 
body_4.GetAssets().push_back(body_4_15_level) 

# Visualization shape 
body_4_16_shape = chrono.ChObjShapeFile() 
body_4_16_shape.SetFilename(shapes_dir +'body_4_16.obj') 
body_4_16_level = chrono.ChAssetLevel() 
body_4_16_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342174,0.00249213119387069)) 
body_4_16_level.GetFrame().SetRot(chrono.ChQuaternionD(0.499218139648065,-0.500780639648065,0.499218139648065,0.500780639648065)) 
body_4_16_level.GetAssets().push_back(body_4_16_shape) 
body_4.GetAssets().push_back(body_4_16_level) 

# Collision shapes 
body_4.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_4.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.000276646091387934,-0.0177501732085428,0.00399213119387072),mr)
body_4.GetCollisionModel().BuildModel()
body_4.SetCollide(True)

exported_items.append(body_4)



# Rigid body part
body_5= chrono.ChBodyAuxRef()
body_5.SetName('Tibia-2')
body_5.SetPos(chrono.ChVectorD(-0.231967742175922,-0.00132652747527695,0.000137255368887401))
body_5.SetRot(chrono.ChQuaternionD(0.872243022401109,4.43849929013048e-16,1.84536666723215e-16,-0.489072704076376))
body_5.SetMass(0.0749832765369386)
body_5.SetInertiaXX(chrono.ChVectorD(3.01244345268189e-05,0.000105187880008893,0.000108478866134613))
body_5.SetInertiaXY(chrono.ChVectorD(-1.65269886560117e-05,3.08955817458714e-07,1.20961418011772e-07))
body_5.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0159616525902916,-0.0203081631545103,0.000240966541478178),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0784186804226129,-0.0590194226200942,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.978768189076E-16,-0.489072704076376,0.872243022401109,-5.33086203055209E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_5.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0643520251386334,-0.0666436740011607,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.978768189076E-16,-0.489072704076376,0.872243022401109,-5.33086203055209E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_5.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155226,-0.0076436740011607,0.0184827446311129)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(8.99345850968001E-18,-0.707106781186547,0.707106781186548,-4.30533367154171E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_5.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155225,0.0126063259988393,-0.00201725536888713)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0,0,0,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_5.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0119591321615523,0.0211063259988393,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.28452087798901E-31,8.89998639252171E-32,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_5.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00804086783844774,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.28452087798901E-31,8.89998639252171E-32,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_5.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00704086783844773,0.0211063259988393,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.28452087798901E-31,8.89998639252171E-32,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_5.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0129591321615523,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.28452087798901E-31,8.89998639252171E-32,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_5.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00750913216155227,-0.00258367400116066,-0.00976725536888713)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,-8.89998639252171E-32,-2.28452087798901E-31)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_5.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00249086783844774,-0.00258367400116066,-0.00976725536888713)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,-8.89998639252171E-32,-2.28452087798901E-31)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_5.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155225,0.0126063259988393,-0.0184172553688871)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.28452087798901E-31,8.89998639252171E-32,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_5.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155226,0.0126063259988393,-1.72553688871264E-05)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_5.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,-0.00764367400116064,-0.0160172553688871)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.94392336003207E-17,0.707106781186548,-0.707106781186547,-9.81307786677363E-18)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_5.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0106052050870878,-0.0516436740011606,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(4.30398771524371E-16,-0.236184670402587,0.97170818740341,-4.3320710589814E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_5.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_5.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_5.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.00245913216155226,-0.00664367400116065,0.00148274463111287),mr)
body_5.GetCollisionModel().BuildModel()
body_5.SetCollide(True)

exported_items.append(body_5)



# Rigid body part
body_6= chrono.ChBodyAuxRef()
body_6.SetName('Femur-2')
body_6.SetPos(chrono.ChVectorD(-0.156999826791457,0.0174633539086122,-0.00243213119387044))
body_6.SetRot(chrono.ChQuaternionD(0.707106781186548,7.85046229341887e-17,2.35513868802566e-16,-0.707106781186547))
body_6.SetMass(0.0560807503629222)
body_6.SetInertiaXX(chrono.ChVectorD(1.23274187765565e-05,3.18822602126284e-05,2.76768873045793e-05))
body_6.SetInertiaXY(chrono.ChVectorD(-2.38080518617806e-06,-7.65814915677935e-07,-2.22070792786802e-07))
body_6.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.000733373824915609,-0.0207493752198647,0.00315852037890482),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0210078688061293)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.138086909818538,0.138086909818538,-0.693492613757902,0.693492613757902)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_6.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0159078688061293)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.138086909818538,0.138086909818538,-0.693492613757902,0.693492613757902)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_6.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0221021311938707)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.00110485434560396,-0.001104854345604,0.707105918018563,0.707105918018563)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_6.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0216521311938707)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.706001063672959,-2.72233131884461E-17,1.41451189427848E-17,0.708210772364167)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_6.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0185078688061293)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,0.707106781186547,-4.35788199605262E-33,-4.41588504004812E-17)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_6.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387937,0.00149982679145716,0.0024921311938707)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_6.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387937,0.00149982679145716,-0.0159078688061293)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(8.71576399210525E-33,-8.71576399210525E-33,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_6.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.00922335390861206,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(8.71576399210525E-33,-8.71576399210525E-33,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_6.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0102233539086121,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(8.71576399210525E-33,-8.71576399210525E-33,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_6.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0107766460913879,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(8.71576399210525E-33,-8.71576399210525E-33,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_6.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387937,0.00149982679145716,0.000492131193870702)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0,0,0,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_6.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00522664609138794,-0.0136901732085428,-0.0072578688061293)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,8.71576399210525E-33,8.71576399210525E-33)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_6.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00477335390861206,-0.0136901732085428,-0.0072578688061293)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,8.71576399210525E-33,8.71576399210525E-33)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_6.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00977664609138794,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(8.71576399210525E-33,-8.71576399210525E-33,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_6.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_4_15_shape = chrono.ChObjShapeFile() 
body_4_15_shape.SetFilename(shapes_dir +'body_4_15.obj') 
body_4_15_level = chrono.ChAssetLevel() 
body_4_15_level.GetFrame().SetPos(chrono.ChVectorD(-0.000259497674868783,-0.0399751463835917,0.0024921311938707)) 
body_4_15_level.GetFrame().SetRot(chrono.ChQuaternionD(0.99999877929613,0,0,0.00156249999999996)) 
body_4_15_level.GetAssets().push_back(body_4_15_shape) 
body_6.GetAssets().push_back(body_4_15_level) 

# Visualization shape 
body_4_16_shape = chrono.ChObjShapeFile() 
body_4_16_shape.SetFilename(shapes_dir +'body_4_16.obj') 
body_4_16_level = chrono.ChAssetLevel() 
body_4_16_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0024921311938707)) 
body_4_16_level.GetFrame().SetRot(chrono.ChQuaternionD(0.499218139648065,-0.500780639648065,0.499218139648065,0.500780639648065)) 
body_4_16_level.GetAssets().push_back(body_4_16_shape) 
body_6.GetAssets().push_back(body_4_16_level) 

# Collision shapes 
body_6.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_6.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.000276646091387934,-0.0177501732085428,0.0039921311938707),mr)
body_6.GetCollisionModel().BuildModel()
body_6.SetCollide(True)

exported_items.append(body_6)



# Rigid body part
body_7= chrono.ChBodyAuxRef()
body_7.SetName('Hip-3')
body_7.SetPos(chrono.ChVectorD(-0.0976796301758524,0.0177400000000002,-0.158679630175923))
body_7.SetRot(chrono.ChQuaternionD(0.923879532511339,5.31016363680581e-16,0.382683432364963,-2.53092883266207e-16))
body_7.SetMass(0.0187191370104082)
body_7.SetInertiaXX(chrono.ChVectorD(8.94046148663507e-06,9.62220940012942e-06,8.00359530179307e-06))
body_7.SetInertiaXY(chrono.ChVectorD(3.18739690085381e-07,1.15018100669965e-06,-3.43276308382587e-07))
body_7.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000962714713021327,0.000962714691254139,0.0260087993424653),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(1.38777878078145E-17,0.01961,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1.44444745829045E-34,-2.77555756156291E-17,1,-5.20417042793047E-18)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_7.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(1.38777878078145E-17,0.01916,0.052)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.5,0.5,-0.5,-0.5)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_7.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(6.93889390390723E-18,-0.021,0.052)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_7.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(-6.93889390390723E-18,3.46944695195361E-18,0.052)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(3.67990420004013E-18,0.707106781186548,-0.707106781186547,3.67990420004013E-18)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_7.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(1.38777878078145E-17,-0.0184,0.052)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-3.107101992451E-31,-0.248882088995415,2.2709255814125E-31,0.968533791758077)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_7.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_7.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(6.93889390390723E-18,-0.0235,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-4.50120647736803E-32,-0.285333923385196,7.71635396120233E-32,0.958428167452111)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_7.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-0.01961,-6.93889390390723E-18,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,5.88784672006416E-17,-5.88784672006416E-17,0.707106781186547)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_7.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-0.01916,-6.93889390390723E-18,2.77555756156289E-17)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,7.85046229341884E-17,-0.707106781186547,-7.8504622934189E-17)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_7.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.021,3.46944695195361E-18,0)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,0,0,0.707106781186547)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_7.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0184,3.46944695195361E-18,0)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,0,0,-0.707106781186547)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_7.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0235,3.46944695195361E-18,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.677711056485583,0.201761552128235,0.201761552128235,-0.677711056485583)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_7.GetAssets().push_back(body_3_1_level) 

exported_items.append(body_7)



# Rigid body part
body_8= chrono.ChBodyAuxRef()
body_8.SetName('Hip-5')
body_8.SetPos(chrono.ChVectorD(0.1555,0.0177399999999999,1.73472347597681e-17))
body_8.SetRot(chrono.ChQuaternionD(0.707106781186548,-2.41328582689849e-16,-0.707106781186547,-6.60190330191666e-16))
body_8.SetMass(0.0187191370104082)
body_8.SetInertiaXX(chrono.ChVectorD(7.32184738751416e-06,9.62220940012942e-06,9.62220940091397e-06))
body_8.SetInertiaXY(chrono.ChVectorD(-4.68116001770688e-07,4.68433092420373e-07,1.73500091852125e-08))
body_8.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000962714713021328,0.00096271469125414,0.0260087993424653),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(7.17434579624321E-18,0.01961,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.77555756156289E-17,-2.77555756156289E-17,1,-5.20417042793041E-18)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_8.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(7.44090778007757E-18,0.01916,0.052)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.5,0.5,-0.5,-0.5)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_8.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(-8.36136715420869E-19,-0.021,0.052)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_8.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(-1.73472347597685E-18,0,0.052)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(2.52407373336373E-18,0.707106781186548,-0.707106781186547,3.83398553251705E-18)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_8.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(-4.02455846426665E-19,-0.0184,0.052)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0,-0.248882088995415,-8.48254213775768E-33,0.968533791758077)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_8.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_8.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-4.02455846426669E-19,-0.0235,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0,-0.285333923385196,-4.71537514937688E-32,0.958428167452111)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_8.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-0.01961,3.46944695195361E-18,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,7.8504622934189E-17,-5.88784672006418E-17,0.707106781186547)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_8.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-0.01916,3.46944695195361E-18,0)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,5.88784672006417E-17,-0.707106781186547,-7.85046229341888E-17)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_8.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.021,0,0)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,2.72733146710864E-31,-2.61107397805583E-31,0.707106781186547)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_8.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0184,0,0)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-2.61107397805583E-31,-2.72733146710864E-31,-0.707106781186547)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_8.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0235,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.677711056485583,0.201761552128235,0.201761552128235,-0.677711056485583)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_8.GetAssets().push_back(body_3_1_level) 

exported_items.append(body_8)



# Rigid body part
body_9= chrono.ChBodyAuxRef()
body_9.SetName('Tibia-4')
body_9.SetPos(chrono.ChVectorD(0.15184754341261,-0.00132652747527714,0.212653435008421))
body_9.SetRot(chrono.ChQuaternionD(0.333792953668955,-0.451844261206113,0.805847475772169,-0.187160021072023))
body_9.SetMass(0.0749832765369386)
body_9.SetInertiaXX(chrono.ChVectorD(9.75020359760893e-05,5.72869200181602e-05,8.90022246760761e-05))
body_9.SetInertiaXY(chrono.ChVectorD(2.39332798175361e-05,-1.47977454126149e-05,3.15654431838859e-05))
body_9.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0159616525902916,-0.0203081631545103,0.000240966541478178),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0784186804226129,-0.0590194226200942,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.70477821746741E-16,-0.489072704076377,0.872243022401109,-5.47569748891474E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_9.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0643520251386335,-0.0666436740011607,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.70477821746741E-16,-0.489072704076377,0.872243022401109,-5.47569748891474E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_9.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,-0.0076436740011607,0.0184827446311129)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(0,-0.707106781186547,0.707106781186548,-1.96261557335472E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_9.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-0.00201725536888712)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0,0,0,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_9.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0119591321615523,0.0211063259988393,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,-5.88784672006416E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_9.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00804086783844774,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,-5.88784672006416E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_9.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00704086783844773,0.0211063259988393,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,-5.88784672006416E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_9.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0129591321615523,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,-5.88784672006416E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_9.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00750913216155227,-0.00258367400116069,-0.00976725536888713)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,5.88784672006416E-17,1.96261557335472E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_9.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00249086783844774,-0.00258367400116066,-0.0097672553688871)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,5.88784672006416E-17,1.96261557335472E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_9.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-0.0184172553688871)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,-5.88784672006416E-17,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_9.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-1.72553688871369E-05)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_9.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,-0.00764367400116067,-0.0160172553688871)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,-0.707106781186547,0.707106781186548,1.96261557335472E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_9.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0106052050870878,-0.0516436740011607,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(4.21226908883428E-16,-0.236184670402587,0.97170818740341,-4.40326225099495E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_9.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_9.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_9.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.00245913216155228,-0.00664367400116068,0.00148274463111286),mr)
body_9.GetCollisionModel().BuildModel()
body_9.SetCollide(True)

exported_items.append(body_9)



# Rigid body part
body_10= chrono.ChBodyAuxRef()
body_10.SetName('Femur-5')
body_10.SetPos(chrono.ChVectorD(0.156999826791457,0.017463353908612,0.00243213119387071))
body_10.SetRot(chrono.ChQuaternionD(-2.59892170246969e-16,0.707106781186548,-0.707106781186547,-1.54390824504435e-16))
body_10.SetMass(0.0560807503629222)
body_10.SetInertiaXX(chrono.ChVectorD(1.23274187765565e-05,3.18822602126284e-05,2.76768873045792e-05))
body_10.SetInertiaXY(chrono.ChVectorD(2.38080518617805e-06,7.65814915677937e-07,-2.22070792786804e-07))
body_10.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00073337382491561,-0.0207493752198647,0.00315852037890482),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0210078688061293)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.138086909818538,0.138086909818538,-0.693492613757902,0.693492613757902)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_10.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0159078688061293)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.138086909818538,0.138086909818538,-0.693492613757902,0.693492613757902)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_10.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0221021311938707)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.00110485434560398,-0.00110485434560402,0.707105918018563,0.707105918018563)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_10.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0216521311938707)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.706001063672959,-2.72233131884461E-17,1.41451189427847E-17,0.708210772364167)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_10.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0185078688061293)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,0.707106781186547,1.96261557335472E-17,-2.4532694666934E-17)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_10.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387944,0.00149982679145716,0.0024921311938707)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_10.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387944,0.00149982679145716,-0.0159078688061293)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_10.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.00922335390861206,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_10.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0102233539086121,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_10.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0107766460913879,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_10.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387944,0.00149982679145716,0.000492131193870702)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.77555756156289E-17,3.421138828918E-49,-1.23259516440783E-32,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_10.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00522664609138794,-0.0136901732085428,-0.0072578688061293)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,-1.96261557335472E-17,1.96261557335472E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_10.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00477335390861205,-0.0136901732085428,-0.0072578688061293)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,-1.96261557335472E-17,1.96261557335472E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_10.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00977664609138794,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.96261557335472E-17,-1.96261557335472E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_10.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_4_15_shape = chrono.ChObjShapeFile() 
body_4_15_shape.SetFilename(shapes_dir +'body_4_15.obj') 
body_4_15_level = chrono.ChAssetLevel() 
body_4_15_level.GetFrame().SetPos(chrono.ChVectorD(-0.000259497674868794,-0.0399751463835917,0.0024921311938707)) 
body_4_15_level.GetFrame().SetRot(chrono.ChQuaternionD(0.99999877929613,1.23259365977414E-32,1.92592994438722E-35,0.00156249999999999)) 
body_4_15_level.GetAssets().push_back(body_4_15_shape) 
body_10.GetAssets().push_back(body_4_15_level) 

# Visualization shape 
body_4_16_shape = chrono.ChObjShapeFile() 
body_4_16_shape.SetFilename(shapes_dir +'body_4_16.obj') 
body_4_16_level = chrono.ChAssetLevel() 
body_4_16_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0024921311938707)) 
body_4_16_level.GetFrame().SetRot(chrono.ChQuaternionD(0.499218139648065,-0.500780639648065,0.499218139648065,0.500780639648065)) 
body_4_16_level.GetAssets().push_back(body_4_16_shape) 
body_10.GetAssets().push_back(body_4_16_level) 

# Collision shapes 
body_10.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_10.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.000276646091387941,-0.0177501732085428,0.0039921311938707),mr)
body_10.GetCollisionModel().BuildModel()
body_10.SetCollide(True)

exported_items.append(body_10)



# Rigid body part
body_11= chrono.ChBodyAuxRef()
body_11.SetName('Tibia-3')
body_11.SetPos(chrono.ChVectorD(-0.151847543412551,-0.00132652747527696,-0.212653435008463))
body_11.SetRot(chrono.ChQuaternionD(0.805847475772123,0.187160021072085,-0.333792953669066,-0.451844261206088))
body_11.SetMass(0.0749832765369386)
body_11.SetInertiaXX(chrono.ChVectorD(4.62224961584775e-05,0.000108566459835783,8.90022246760655e-05))
body_11.SetInertiaXY(chrono.ChVectorD(-2.32880204231334e-06,3.48406057056257e-05,1.21764804326916e-06))
body_11.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0159616525902916,-0.0203081631545103,0.000240966541478178),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0784186804226129,-0.0590194226200942,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.044323066249E-16,-0.489072704076377,0.872243022401109,-5.91603156458848E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_11.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0643520251386335,-0.0666436740011607,0.0149827446311128)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.044323066249E-16,-0.489072704076377,0.872243022401109,-5.91603156458848E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_11.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,-0.0076436740011607,0.0184827446311129)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,0.707106781186548,-0.707106781186547,5.88784672006415E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_11.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-0.00201725536888714)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(9.71445146547012E-17,5.55111512312578E-17,5.55111512312578E-17,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_11.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0119591321615523,0.0211063259988394,0.0144827446311128)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(2.94392336003208E-17,1.0794385653451E-16,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_11.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00804086783844774,-0.0203936740011607,0.0144827446311128)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(2.94392336003208E-17,1.0794385653451E-16,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_11.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00704086783844773,0.0211063259988394,0.0144827446311128)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(2.94392336003208E-17,1.0794385653451E-16,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_11.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0129591321615523,-0.0203936740011607,0.0144827446311128)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(2.94392336003208E-17,1.0794385653451E-16,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_11.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00750913216155227,-0.00258367400116069,-0.00976725536888713)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,1.0794385653451E-16,-2.94392336003208E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_11.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00249086783844771,-0.00258367400116066,-0.00976725536888716)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,1.0794385653451E-16,-2.94392336003208E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_11.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-0.0184172553688872)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(2.94392336003208E-17,1.0794385653451E-16,0.707106781186547,0.707106781186548)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_11.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-1.72553688871507E-05)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_11.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.0024591321615523,-0.00764367400116064,-0.0160172553688871)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(5.88784672006416E-17,0.707106781186548,-0.707106781186547,5.88784672006416E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_11.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0106052050870879,-0.0516436740011606,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(3.40315948668025E-16,-0.236184670402587,0.97170818740341,-4.59992549535329E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_11.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_11.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_11.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.00245913216155228,-0.00664367400116065,0.00148274463111285),mr)
body_11.GetCollisionModel().BuildModel()
body_11.SetCollide(True)

exported_items.append(body_11)



# Rigid body part
body_12= chrono.ChBodyAuxRef()
body_12.SetName('Tibia-5')
body_12.SetPos(chrono.ChVectorD(0.231967742175922,-0.00132652747527721,-0.000137255368887128))
body_12.SetRot(chrono.ChQuaternionD(3.07937645915198e-16,-0.489072704076376,0.872243022401109,1.88573231035644e-16))
body_12.SetMass(0.0749832765369386)
body_12.SetInertiaXX(chrono.ChVectorD(0.000103001570666625,3.23107438690869e-05,0.000108478866134613))
body_12.SetInertiaXY(chrono.ChVectorD(2.07959928259302e-05,-3.316904818504e-08,3.30129029368096e-07))
body_12.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0159616525902916,-0.0203081631545103,0.000240966541478178),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0784186804226129,-0.0590194226200941,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-3.07937645915198E-16,-0.489072704076376,0.872243022401109,1.88573231035644E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_12.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0643520251386334,-0.0666436740011607,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-3.07937645915198E-16,-0.489072704076376,0.872243022401109,1.88573231035644E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_12.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155225,-0.00764367400116067,0.0184827446311129)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(8.83177008009622E-17,-0.707106781186547,0.707106781186548,-4.90653893338679E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_12.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155226,0.0126063259988393,-0.00201725536888713)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(0,0,0,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_12.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0119591321615523,0.0211063259988394,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.46533782117269E-31,-1.32370665630099E-31,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_12.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00804086783844775,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.46533782117269E-31,-1.32370665630099E-31,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_12.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00704086783844775,0.0211063259988393,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.46533782117269E-31,-1.32370665630099E-31,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_12.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0129591321615523,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.46533782117269E-31,-1.32370665630099E-31,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_12.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00750913216155226,-0.00258367400116069,-0.00976725536888713)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,1.32370665630099E-31,-1.46533782117269E-31)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_12.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00249086783844776,-0.00258367400116069,-0.00976725536888713)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,1.32370665630099E-31,-1.46533782117269E-31)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_12.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155226,0.0126063259988393,-0.0184172553688871)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.46533782117269E-31,-1.32370665630099E-31,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_12.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155226,0.0126063259988393,-1.72553688871264E-05)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_12.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,-0.00764367400116067,-0.0160172553688871)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(2.94392336003207E-17,-0.707106781186547,0.707106781186548,9.81307786677368E-18)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_12.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0106052050870878,-0.0516436740011606,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(4.3039877152437E-16,-0.236184670402587,0.97170818740341,-4.3320710589814E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_12.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_12.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_12.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.00245913216155226,-0.00664367400116065,0.00148274463111287),mr)
body_12.GetCollisionModel().BuildModel()
body_12.SetCollide(True)

exported_items.append(body_12)



# Rigid body part
body_13= chrono.ChBodyAuxRef()
body_13.SetName('Tibia-6')
body_13.SetPos(chrono.ChVectorD(0.151653435008421,-0.00132652747527703,-0.21284754341261))
body_13.SetRot(chrono.ChQuaternionD(-0.333792953668956,-0.451844261206112,0.805847475772169,0.187160021072023))
body_13.SetMass(0.0749832765369386)
body_13.SetInertiaXX(chrono.ChVectorD(9.76105585855413e-05,5.66496811012013e-05,8.95309409835829e-05))
body_13.SetInertiaXY(chrono.ChVectorD(2.38272353619837e-05,1.45558266001057e-05,-3.14175393474913e-05))
body_13.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0159616525902916,-0.0203081631545103,0.000240966541478179),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0784186804226129,-0.0590194226200941,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.82655754927769E-16,-0.489072704076376,0.872243022401109,-5.04106051968017E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_13.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0643520251386334,-0.0666436740011606,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.82655754927769E-16,-0.489072704076376,0.872243022401109,-5.04106051968017E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_13.GetAssets().push_back(body_1_1_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,-0.00764367400116067,0.0184827446311129)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(-3.92523114670944E-17,0.707106781186548,-0.707106781186547,3.92523114670944E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_13.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-0.00201725536888712)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(1.38777878078144E-17,2.77555756156289E-17,3.46667389989703E-33,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_13.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0119591321615523,0.0211063259988393,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-9.8130778667736E-18,2.94392336003208E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_13.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00804086783844771,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-9.8130778667736E-18,2.94392336003208E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_13.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00704086783844773,0.0211063259988393,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-9.8130778667736E-18,2.94392336003208E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_13.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0129591321615523,-0.0203936740011607,0.0144827446311129)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-9.8130778667736E-18,2.94392336003208E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_13.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00750913216155227,-0.00258367400116066,-0.0097672553688871)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-0.707106781186548,2.94392336003208E-17,9.81307786677363E-18)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_13.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00249086783844774,-0.00258367400116066,-0.00976725536888715)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-0.707106781186548,2.94392336003208E-17,9.81307786677363E-18)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_13.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-0.0184172553688871)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(-9.8130778667736E-18,2.94392336003208E-17,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_13.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,0.0126063259988393,-1.72553688871369E-05)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_13.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_12_shape = chrono.ChObjShapeFile() 
body_1_12_shape.SetFilename(shapes_dir +'body_1_12.obj') 
body_1_12_level = chrono.ChAssetLevel() 
body_1_12_level.GetFrame().SetPos(chrono.ChVectorD(0.00245913216155227,-0.00764367400116067,-0.0160172553688871)) 
body_1_12_level.GetFrame().SetRot(chrono.ChQuaternionD(1.96261557335472E-17,0.707106781186548,-0.707106781186547,-1.96261557335472E-17)) 
body_1_12_level.GetAssets().push_back(body_1_12_shape) 
body_13.GetAssets().push_back(body_1_12_level) 

# Visualization shape 
body_1_1_shape = chrono.ChObjShapeFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1_1_level = chrono.ChAssetLevel() 
body_1_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0106052050870878,-0.0516436740011606,0.0149827446311129)) 
body_1_1_level.GetFrame().SetRot(chrono.ChQuaternionD(4.14671467404817E-16,-0.236184670402587,0.97170818740341,-4.13355905027694E-16)) 
body_1_1_level.GetAssets().push_back(body_1_1_shape) 
body_13.GetAssets().push_back(body_1_1_level) 

# Collision shapes 
body_13.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_13.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(0.00245913216155228,-0.00664367400116065,0.00148274463111286),mr)
body_13.GetCollisionModel().BuildModel()
body_13.SetCollide(True)

exported_items.append(body_13)



# Rigid body part
body_14= chrono.ChBodyAuxRef()
body_14.SetName('Hip-2')
body_14.SetPos(chrono.ChVectorD(-0.155500000000001,0.0177400000000002,1.80411241501588e-16))
body_14.SetRot(chrono.ChQuaternionD(0.707106781186547,2.8961915739331e-16,0.707106781186548,-7.04251349475003e-17))
body_14.SetMass(0.0187191370104082)
body_14.SetInertiaXX(chrono.ChVectorD(7.32184738751416e-06,9.62220940012942e-06,9.62220940091397e-06))
body_14.SetInertiaXY(chrono.ChVectorD(4.68116001770687e-07,4.68433092420373e-07,-1.73500091852124e-08))
body_14.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000962714713021328,0.00096271469125414,0.0260087993424653),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-6.63320543741877E-18,0.01961,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(2.77555756156289E-17,-2.77555756156289E-17,1,-5.20417042793043E-18)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_14.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-6.77269965911904E-18,0.01916,0.052)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.5,0.5,-0.5,-0.5)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_14.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(-8.3613671542082E-19,-0.021,0.052)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_14.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(-1.73472347597681E-18,3.46944695195361E-18,0.052)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(2.52407373336359E-18,0.707106781186548,-0.707106781186547,3.83398553251678E-18)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_14.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(-4.02455846426643E-19,-0.0184,0.052)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.27264033005026E-32,-0.248882088995415,-6.74750208818996E-32,0.968533791758077)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_14.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_14.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-4.02455846426643E-19,-0.0235,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1.28605899353372E-32,-0.285333923385196,-4.24652946539978E-32,0.958428167452111)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_14.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-0.01961,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,7.85046229341887E-17,-5.88784672006417E-17,0.707106781186547)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_14.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-0.01916,0,0)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-2.8961915739331E-16,-0.707106781186548,7.04251349475003E-17)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_14.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.021,0,0)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-5.26381545523345E-33,-1.09852983360496E-31,0.707106781186547)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_14.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0184,0,0)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-1.09852983360496E-31,5.26381545523345E-33,-0.707106781186547)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_14.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0235,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.677711056485583,0.201761552128235,0.201761552128235,-0.677711056485583)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_14.GetAssets().push_back(body_3_1_level) 

exported_items.append(body_14)



# Rigid body part
body_15= chrono.ChBodyAuxRef()
body_15.SetName('Hip-6')
body_15.SetPos(chrono.ChVectorD(0.0976796301758963,0.0177400000000002,-0.158679630175896))
body_15.SetRot(chrono.ChQuaternionD(0.923879532511287,2.34714092062633e-16,-0.38268343236509,-5.5305079164958e-16))
body_15.SetMass(0.0187191370104082)
body_15.SetInertiaXX(chrono.ChVectorD(8.0035953017937e-06,9.62220940012942e-06,8.94046148663445e-06))
body_15.SetInertiaXY(chrono.ChVectorD(-3.43276308382501e-07,-1.1501810066999e-06,-3.18739690085475e-07))
body_15.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000962714713021328,0.00096271469125414,0.0260087993424653),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,0.01961,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0,-2.77555756156289E-17,1,-5.20417042793045E-18)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_15.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(0,0.01916,0.052)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.5,0.5,-0.5,-0.5)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_15.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0,-0.021,0.052)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_15.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(1.38777878078145E-17,0,0.052)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(3.67990420003999E-18,0.707106781186548,-0.707106781186547,3.67990420003999E-18)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_15.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0,-0.0184,0.052)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0,-0.248882088995416,-1.14537629704524E-31,0.968533791758077)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_15.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_15.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0,-0.0235,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-1.28605899353372E-31,-0.285333923385196,1.92908849030058E-32,0.958428167452111)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_15.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-0.01961,-6.93889390390723E-18,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,7.85046229341886E-17,-5.88784672006416E-17,0.707106781186547)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_15.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-0.01916,-6.93889390390723E-18,2.77555756156289E-17)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,5.88784672006418E-17,-0.707106781186548,-7.85046229341887E-17)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_15.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.021,-3.46944695195361E-18,0)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-1.04589167905263E-31,-1.04589167905263E-31,0.707106781186547)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_15.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0184,0,0)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-1.04589167905263E-31,1.04589167905263E-31,-0.707106781186547)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_15.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0235,0,2.77555756156289E-17)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.677711056485583,0.201761552128235,0.201761552128235,-0.677711056485583)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_15.GetAssets().push_back(body_3_1_level) 

exported_items.append(body_15)



# Rigid body part
body_16= chrono.ChBodyAuxRef()
body_16.SetName('Femur-4')
body_16.SetPos(chrono.ChVectorD(0.0970203914108197,0.0174633539086121,0.161459944330662))
body_16.SetRot(chrono.ChQuaternionD(-0.270598050073098,0.653281482438189,-0.653281482438189,0.270598050073098))
body_16.SetMass(0.0560807503629222)
body_16.SetInertiaXX(chrono.ChVectorD(1.23274187765565e-05,2.9557502965817e-05,3.00016445513906e-05))
body_16.SetInertiaXY(chrono.ChVectorD(2.22499641184028e-06,-1.14197057182093e-06,-2.10268645402459e-06))
body_16.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.000733373824915608,-0.0207493752198646,0.00315852037890482),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0210078688061293)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.138086909818538,-0.138086909818538,0.693492613757902,-0.693492613757902)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_16.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0159078688061293)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.138086909818538,-0.138086909818538,0.693492613757902,-0.693492613757902)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_16.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0221021311938707)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.00110485434560398,-0.001104854345604,0.707105918018563,0.707105918018563)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_16.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0216521311938707)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.706001063672959,-8.49647154582542E-18,-8.52306460352102E-18,0.708210772364167)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_16.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0185078688061293)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,0.707106781186547,-4.63332289777667E-19,-4.07605912900919E-17)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_16.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387941,0.00149982679145716,0.00249213119387069)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_16.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387941,0.00149982679145716,-0.0159078688061293)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(4.63332289777672E-19,-3.39825911038926E-18,0.707106781186547,0.707106781186548)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_16.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.00922335390861206,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(4.63332289777672E-19,-3.39825911038926E-18,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_16.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0102233539086121,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(4.63332289777672E-19,-3.39825911038926E-18,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_16.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0107766460913879,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(4.63332289777672E-19,-3.39825911038926E-18,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_16.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387941,0.00149982679145719,0.00049213119387069)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.07530665714073E-18,-2.7305574652297E-18,1.70002322558895E-35,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_16.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00522664609138794,-0.0136901732085428,-0.00725786880612932)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,-3.39825911038926E-18,-4.63332289777672E-19)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_16.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00477335390861206,-0.0136901732085428,-0.00725786880612932)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,-3.39825911038926E-18,-4.63332289777672E-19)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_16.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00977664609138794,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(4.63332289777672E-19,-3.39825911038926E-18,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_16.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_4_15_shape = chrono.ChObjShapeFile() 
body_4_15_shape.SetFilename(shapes_dir +'body_4_15.obj') 
body_4_15_level = chrono.ChAssetLevel() 
body_4_15_level.GetFrame().SetPos(chrono.ChVectorD(-0.000259497674868787,-0.0399751463835916,0.00249213119387071)) 
body_4_15_level.GetFrame().SetRot(chrono.ChQuaternionD(0.99999877929613,4.57397791517308E-21,-2.92734229228843E-18,0.00156249999999996)) 
body_4_15_level.GetAssets().push_back(body_4_15_shape) 
body_16.GetAssets().push_back(body_4_15_level) 

# Visualization shape 
body_4_16_shape = chrono.ChObjShapeFile() 
body_4_16_shape.SetFilename(shapes_dir +'body_4_16.obj') 
body_4_16_level = chrono.ChAssetLevel() 
body_4_16_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.00249213119387071)) 
body_4_16_level.GetFrame().SetRot(chrono.ChQuaternionD(0.499218139648065,-0.500780639648065,0.499218139648065,0.500780639648065)) 
body_4_16_level.GetAssets().push_back(body_4_16_shape) 
body_16.GetAssets().push_back(body_4_16_level) 

# Collision shapes 
body_16.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_16.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.000276646091387937,-0.0177501732085428,0.00399213119387069),mr)
body_16.GetCollisionModel().BuildModel()
body_16.SetCollide(True)

exported_items.append(body_16)



# Rigid body part
body_17= chrono.ChBodyAuxRef()
body_17.SetName('Femur-3')
body_17.SetPos(chrono.ChVectorD(-0.097020391410775,0.0174633539086123,-0.161459944330689))
body_17.SetRot(chrono.ChQuaternionD(0.653281482438151,0.270598050073188,-0.270598050073188,-0.653281482438151))
body_17.SetMass(0.0560807503629222)
body_17.SetInertiaXX(chrono.ChVectorD(1.23274187765565e-05,2.95575029658159e-05,3.00016445513918e-05))
body_17.SetInertiaXY(chrono.ChVectorD(-2.22499641183996e-06,1.14197057182155e-06,-2.10268645402448e-06))
body_17.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.000733373824915609,-0.0207493752198647,0.00315852037890482),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0210078688061293)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.138086909818538,-0.138086909818538,0.693492613757902,-0.693492613757902)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_17.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0159078688061293)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.138086909818538,-0.138086909818538,0.693492613757902,-0.693492613757902)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_17.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0221021311938707)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.00110485434560398,-0.001104854345604,0.707105918018563,0.707105918018563)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_17.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0216521311938707)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.706001063672959,-6.58285182829717E-18,-6.60345545858386E-18,0.708210772364167)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_17.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0185078688061293)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,0.707106781186547,8.71576399210525E-33,-4.41588504004812E-17)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_17.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387941,0.00149982679145716,0.00249213119387071)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_17.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387941,0.00149982679145716,-0.0159078688061293)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(4.35788199605264E-33,3.05051739723684E-32,0.707106781186547,0.707106781186548)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_17.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.00922335390861206,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(4.35788199605264E-33,3.05051739723684E-32,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_17.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0102233539086121,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(4.35788199605264E-33,3.05051739723684E-32,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_17.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0107766460913879,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(4.35788199605264E-33,3.05051739723684E-32,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_17.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387941,0.00149982679145716,0.000492131193870704)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(2.46519032881566E-32,1.84889274661175E-32,1.36736175538941E-63,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_17.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00522664609138794,-0.0136901732085428,-0.0072578688061293)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,3.05051739723684E-32,-4.35788199605264E-33)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_17.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00477335390861206,-0.0136901732085428,-0.0072578688061293)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,-0.707106781186547,3.05051739723684E-32,-4.35788199605264E-33)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_17.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00977664609138794,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(4.35788199605264E-33,3.05051739723684E-32,0.707106781186547,0.707106781186548)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_17.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_4_15_shape = chrono.ChObjShapeFile() 
body_4_15_shape.SetFilename(shapes_dir +'body_4_15.obj') 
body_4_15_level = chrono.ChAssetLevel() 
body_4_15_level.GetFrame().SetPos(chrono.ChVectorD(-0.000259497674868787,-0.0399751463835917,0.00249213119387069)) 
body_4_15_level.GetFrame().SetRot(chrono.ChQuaternionD(0.99999877929613,1.69406589450855E-22,-1.08420084899572E-19,0.00156249999999996)) 
body_4_15_level.GetAssets().push_back(body_4_15_shape) 
body_17.GetAssets().push_back(body_4_15_level) 

# Visualization shape 
body_4_16_shape = chrono.ChObjShapeFile() 
body_4_16_shape.SetFilename(shapes_dir +'body_4_16.obj') 
body_4_16_level = chrono.ChAssetLevel() 
body_4_16_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.00249213119387071)) 
body_4_16_level.GetFrame().SetRot(chrono.ChQuaternionD(0.499218139648065,-0.500780639648065,0.499218139648065,0.500780639648065)) 
body_4_16_level.GetAssets().push_back(body_4_16_shape) 
body_17.GetAssets().push_back(body_4_16_level) 

# Collision shapes 
body_17.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_17.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.000276646091387937,-0.0177501732085428,0.0039921311938707),mr)
body_17.GetCollisionModel().BuildModel()
body_17.SetCollide(True)

exported_items.append(body_17)



# Rigid body part
body_18= chrono.ChBodyAuxRef()
body_18.SetName('Femur-6')
body_18.SetPos(chrono.ChVectorD(0.100459944330662,0.0174633539086122,-0.15802039141082))
body_18.SetRot(chrono.ChQuaternionD(-0.270598050073098,-0.653281482438188,0.653281482438189,0.270598050073099))
body_18.SetMass(0.0560807503629222)
body_18.SetInertiaXX(chrono.ChVectorD(1.23274187765565e-05,3.00016445513906e-05,2.9557502965817e-05))
body_18.SetInertiaXY(chrono.ChVectorD(1.14197057182095e-06,2.22499641184029e-06,2.1026864540246e-06))
body_18.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.000733373824915609,-0.0207493752198646,0.00315852037890482),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0210078688061293)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.138086909818538,0.138086909818538,-0.693492613757902,0.693492613757902)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_18.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0159078688061293)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.138086909818538,0.138086909818538,-0.693492613757902,0.693492613757902)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_18.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0221021311938707)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.001104854345604,-0.00110485434560399,0.707105918018563,0.707105918018563)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_18.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.0216521311938707)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.706001063672959,3.50906265104012E-17,-1.04790571462518E-16,0.708210772364167)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_18.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,-0.0185078688061293)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,0.707106781186547,2.05226793299056E-17,-2.88082338627219E-17)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_18.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_1_3_shape = chrono.ChObjShapeFile() 
body_1_3_shape.SetFilename(shapes_dir +'body_1_3.obj') 
body_1_3_level = chrono.ChAssetLevel() 
body_1_3_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387937,0.00149982679145716,0.00249213119387071)) 
body_1_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_1_3_level.GetAssets().push_back(body_1_3_shape) 
body_18.GetAssets().push_back(body_1_3_level) 

# Visualization shape 
body_1_4_shape = chrono.ChObjShapeFile() 
body_1_4_shape.SetFilename(shapes_dir +'body_1_4.obj') 
body_1_4_level = chrono.ChAssetLevel() 
body_1_4_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387937,0.00149982679145716,-0.0159078688061293)) 
body_1_4_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.05226793299056E-17,-1.53506165377592E-17,0.707106781186548,0.707106781186547)) 
body_1_4_level.GetAssets().push_back(body_1_4_shape) 
body_18.GetAssets().push_back(body_1_4_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.00922335390861206,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.05226793299056E-17,-1.53506165377592E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_18.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0102233539086121,-0.0315001732085429,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.05226793299056E-17,-1.53506165377592E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_18.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.0107766460913879,-0.0315001732085428,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.05226793299056E-17,-1.53506165377592E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_18.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_1_8_shape = chrono.ChObjShapeFile() 
body_1_8_shape.SetFilename(shapes_dir +'body_1_8.obj') 
body_1_8_level = chrono.ChAssetLevel() 
body_1_8_level.GetFrame().SetPos(chrono.ChVectorD(-0.000276646091387937,0.00149982679145716,0.000492131193870718)) 
body_1_8_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.53662507715371E-17,3.65720067304938E-18,-1.11022302462516E-16,1)) 
body_1_8_level.GetAssets().push_back(body_1_8_shape) 
body_18.GetAssets().push_back(body_1_8_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(-0.00522664609138794,-0.0136901732085428,-0.0072578688061293)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-0.707106781186548,-1.53506165377592E-17,2.05226793299056E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_18.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_9_shape = chrono.ChObjShapeFile() 
body_1_9_shape.SetFilename(shapes_dir +'body_1_9.obj') 
body_1_9_level = chrono.ChAssetLevel() 
body_1_9_level.GetFrame().SetPos(chrono.ChVectorD(0.00477335390861206,-0.0136901732085428,-0.00725786880612928)) 
body_1_9_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186547,-0.707106781186548,-1.53506165377592E-17,2.05226793299056E-17)) 
body_1_9_level.GetAssets().push_back(body_1_9_shape) 
body_18.GetAssets().push_back(body_1_9_level) 

# Visualization shape 
body_1_5_shape = chrono.ChObjShapeFile() 
body_1_5_shape.SetFilename(shapes_dir +'body_1_5.obj') 
body_1_5_level = chrono.ChAssetLevel() 
body_1_5_level.GetFrame().SetPos(chrono.ChVectorD(-0.00977664609138794,0.00999982679145717,0.0169921311938707)) 
body_1_5_level.GetFrame().SetRot(chrono.ChQuaternionD(-2.05226793299056E-17,-1.53506165377592E-17,0.707106781186548,0.707106781186547)) 
body_1_5_level.GetAssets().push_back(body_1_5_shape) 
body_18.GetAssets().push_back(body_1_5_level) 

# Visualization shape 
body_4_15_shape = chrono.ChObjShapeFile() 
body_4_15_shape.SetFilename(shapes_dir +'body_4_15.obj') 
body_4_15_level = chrono.ChAssetLevel() 
body_4_15_level.GetFrame().SetPos(chrono.ChVectorD(-0.00025949767486879,-0.0399751463835917,0.00249213119387068)) 
body_4_15_level.GetFrame().SetRot(chrono.ChQuaternionD(0.99999877929613,1.38772626473768E-16,3.46944298148425E-18,0.00156249999999998)) 
body_4_15_level.GetAssets().push_back(body_4_15_shape) 
body_18.GetAssets().push_back(body_4_15_level) 

# Visualization shape 
body_4_16_shape = chrono.ChObjShapeFile() 
body_4_16_shape.SetFilename(shapes_dir +'body_4_16.obj') 
body_4_16_level = chrono.ChAssetLevel() 
body_4_16_level.GetFrame().SetPos(chrono.ChVectorD(0.0143123064366119,-0.0629297216342173,0.00249213119387069)) 
body_4_16_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.499218139648065,0.500780639648065,-0.499218139648065,-0.500780639648065)) 
body_4_16_level.GetAssets().push_back(body_4_16_shape) 
body_18.GetAssets().push_back(body_4_16_level) 

# Collision shapes 
body_18.GetCollisionModel().ClearModel()
mr = chrono.ChMatrix33D()
mr[0,0]=-1.80231010491097E-16; mr[1,0]=1; mr[2,0]=0 
mr[0,1]=1; mr[1,1]=0; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=-1 
body_18.GetCollisionModel().AddBox(mymat, 0.01925,0.016,0.0175,chrono.ChVectorD(-0.000276646091387934,-0.0177501732085428,0.0039921311938707),mr)
body_18.GetCollisionModel().BuildModel()
body_18.SetCollide(True)

exported_items.append(body_18)



# Rigid body part
body_19= chrono.ChBodyAuxRef()
body_19.SetName('Hip-4')
body_19.SetPos(chrono.ChVectorD(0.0976796301758964,0.01774,0.158679630175896))
body_19.SetRot(chrono.ChQuaternionD(-0.382683432365091,2.05570733263108e-16,0.923879532511286,5.76295559911791e-16))
body_19.SetMass(0.0187191370104082)
body_19.SetInertiaXX(chrono.ChVectorD(8.94046148663444e-06,9.62220940012942e-06,8.0035953017937e-06))
body_19.SetInertiaXY(chrono.ChVectorD(-3.18739690085476e-07,1.15018100669991e-06,3.432763083825e-07))
body_19.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-0.000962714713021326,0.000962714691254138,0.0260087993424653),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-1.38777878078145E-17,0.01961,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(1.44444745829043E-34,-2.77555756156289E-17,1,-5.20417042793042E-18)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_19.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-1.38777878078145E-17,0.01916,0.052)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(-0.5,0.5,-0.5,-0.5)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_19.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(-1.38777878078145E-17,-0.021,0.052)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_19.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0.052)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(3.6799042000401E-18,0.707106781186548,-0.707106781186547,3.6799042000401E-18)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_19.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(-1.38777878078145E-17,-0.0184,0.052)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(3.10859488622264E-32,-0.248882088995415,-7.71289518336411E-32,0.968533791758077)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_19.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_4_shape = chrono.ChObjShapeFile() 
body_3_4_shape.SetFilename(shapes_dir +'body_3_4.obj') 
body_3_4_level = chrono.ChAssetLevel() 
body_3_4_level.GetFrame().SetPos(chrono.ChVectorD(0,0,0)) 
body_3_4_level.GetFrame().SetRot(chrono.ChQuaternionD(1,0,0,0)) 
body_3_4_level.GetAssets().push_back(body_3_4_shape) 
body_19.GetAssets().push_back(body_3_4_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-1.38777878078145E-17,-0.0235,0.052)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0,-0.285333923385196,-2.57211798706744E-32,0.958428167452111)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_19.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(-0.01961,-3.46944695195361E-18,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,5.88784672006416E-17,-5.88784672006416E-17,0.707106781186547)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_19.GetAssets().push_back(body_3_1_level) 

# Visualization shape 
body_3_2_shape = chrono.ChObjShapeFile() 
body_3_2_shape.SetFilename(shapes_dir +'body_3_2.obj') 
body_3_2_level = chrono.ChAssetLevel() 
body_3_2_level.GetFrame().SetPos(chrono.ChVectorD(-0.01916,-3.46944695195361E-18,-2.77555756156289E-17)) 
body_3_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,7.85046229341887E-17,-0.707106781186547,-7.85046229341886E-17)) 
body_3_2_level.GetAssets().push_back(body_3_2_shape) 
body_19.GetAssets().push_back(body_3_2_level) 

# Visualization shape 
body_3_3_shape = chrono.ChObjShapeFile() 
body_3_3_shape.SetFilename(shapes_dir +'body_3_3.obj') 
body_3_3_level = chrono.ChAssetLevel() 
body_3_3_level.GetFrame().SetPos(chrono.ChVectorD(0.021,0,-2.77555756156289E-17)) 
body_3_3_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,3.4863055968421E-32,-3.4863055968421E-32,0.707106781186547)) 
body_3_3_level.GetAssets().push_back(body_3_3_shape) 
body_19.GetAssets().push_back(body_3_3_level) 

# Visualization shape 
body_3_5_shape = chrono.ChObjShapeFile() 
body_3_5_shape.SetFilename(shapes_dir +'body_3_5.obj') 
body_3_5_level = chrono.ChAssetLevel() 
body_3_5_level.GetFrame().SetPos(chrono.ChVectorD(0.0184,0,-2.77555756156289E-17)) 
body_3_5_level.GetFrame().SetRot(chrono.ChQuaternionD(0.707106781186548,3.4863055968421E-32,3.4863055968421E-32,-0.707106781186547)) 
body_3_5_level.GetAssets().push_back(body_3_5_shape) 
body_19.GetAssets().push_back(body_3_5_level) 

# Visualization shape 
body_3_1_shape = chrono.ChObjShapeFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3_1_level = chrono.ChAssetLevel() 
body_3_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0235,0,0)) 
body_3_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.677711056485583,0.201761552128235,0.201761552128235,-0.677711056485583)) 
body_3_1_level.GetAssets().push_back(body_3_1_shape) 
body_19.GetAssets().push_back(body_3_1_level) 

exported_items.append(body_19)



# Rigid body part
body_20= chrono.ChBodyAuxRef()
body_20.SetName('Foot-3')
body_20.SetPos(chrono.ChVectorD(-0.174931546610825,-0.00132652747527689,-0.235786243760157))
body_20.SetRot(chrono.ChQuaternionD(0.333792953669066,-0.451844261206088,0.805847475772123,-0.187160021072085))
body_20.SetMass(0.00932818757671917)
body_20.SetInertiaXX(chrono.ChVectorD(2.09718093707935e-06,1.19139594044008e-06,1.36874591235429e-06))
body_20.SetInertiaXY(chrono.ChVectorD(-7.76333868838185e-08,-3.23343716350289e-07,1.76078253615457e-07))
body_20.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0846037978523311,-0.0708768966115268,-1.72553688872245e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0987885988031631,-0.0795491716502537,-1.7255368887234E-05)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.680657863238742,-0.191585159162366,-0.680657863238743,-0.191585159162366)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_20.GetAssets().push_back(body_20_1_level) 

# Visualization shape 
body_20_2_shape = chrono.ChObjShapeFile() 
body_20_2_shape.SetFilename(shapes_dir +'body_20_2.obj') 
body_20_2_level = chrono.ChAssetLevel() 
body_20_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0720327070443651,-0.0590889112366741,-1.72553688872479E-05)) 
body_20_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.616768955982473,0.345826625545647,-0.616768955982474,0.345826625545648)) 
body_20_2_level.GetAssets().push_back(body_20_2_shape) 
body_20.GetAssets().push_back(body_20_2_level) 

# Collision shapes 
body_20.GetCollisionModel().ClearModel()
body_20.GetCollisionModel().AddSphere(mymat, 0.0045, chrono.ChVectorD(0.0987885988031631,-0.0795491716502537,-1.7255368887234E-05))
body_20.GetCollisionModel().BuildModel()
body_20.SetCollide(True)

exported_items.append(body_20)



# Rigid body part
body_21= chrono.ChBodyAuxRef()
body_21.SetName('Foot-2')
body_21.SetPos(chrono.ChVectorD(-0.151653435008421,-0.0013265274752771,0.21284754341261))
body_21.SetRot(chrono.ChQuaternionD(0.805847475772168,-0.187160021072023,0.333792953668956,-0.451844261206114))
body_21.SetMass(0.00932818757671917)
body_21.SetInertiaXX(chrono.ChVectorD(1.44259921992284e-06,1.8459776575966e-06,1.36874591235428e-06))
body_21.SetInertiaXY(chrono.ChVectorD(-4.12868037995549e-07,-3.04112106436446e-07,2.07534423694475e-07))
body_21.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0846037978523311,-0.0708768966115268,-1.72553688872235e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0.098788598803163,-0.0795491716502536,-1.72553688871924E-05)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.680657863238742,-0.191585159162366,-0.680657863238743,-0.191585159162366)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_21.GetAssets().push_back(body_20_1_level) 

# Visualization shape 
body_20_2_shape = chrono.ChObjShapeFile() 
body_20_2_shape.SetFilename(shapes_dir +'body_20_2.obj') 
body_20_2_level = chrono.ChAssetLevel() 
body_20_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0720327070443652,-0.0590889112366741,-1.7255368887234E-05)) 
body_20_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.616768955982473,0.345826625545647,-0.616768955982474,0.345826625545648)) 
body_20_2_level.GetAssets().push_back(body_20_2_shape) 
body_21.GetAssets().push_back(body_20_2_level) 

# Collision shapes 
body_21.GetCollisionModel().ClearModel()
body_21.GetCollisionModel().AddSphere(mymat, 0.0045, chrono.ChVectorD(0.098788598803163,-0.0795491716502536,-1.72553688871924E-05))
body_21.GetCollisionModel().BuildModel()
body_21.SetCollide(True)

exported_items.append(body_21)



# Rigid body part
body_22= chrono.ChBodyAuxRef()
body_22.SetName('Foot-5')
body_22.SetPos(chrono.ChVectorD(0.264647963310568,-0.00132652747527714,-0.000102744631112559))
body_22.SetRot(chrono.ChQuaternionD(0.872243022401109,8.5801098595816e-16,1.09787184626005e-16,-0.489072704076376))
body_22.SetMass(0.00932818757671917)
body_22.SetInertiaXX(chrono.ChVectorD(1.31204666306433e-06,1.99496486563108e-06,1.35031126117831e-06))
body_22.SetInertiaXY(chrono.ChVectorD(-4.74093390179693e-07,8.1220860451277e-22,-1.19578142374346e-21))
body_22.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0846037978523311,-0.0708768966115268,-1.72553688872239e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0.0987885988031631,-0.0795491716502536,-1.72553688871985E-05)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.680657863238742,-0.191585159162366,-0.680657863238743,-0.191585159162366)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_22.GetAssets().push_back(body_20_1_level) 

# Visualization shape 
body_20_2_shape = chrono.ChObjShapeFile() 
body_20_2_shape.SetFilename(shapes_dir +'body_20_2.obj') 
body_20_2_level = chrono.ChAssetLevel() 
body_20_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0720327070443652,-0.0590889112366742,-1.72553688872406E-05)) 
body_20_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.616768955982473,0.345826625545647,-0.616768955982474,0.345826625545648)) 
body_20_2_level.GetAssets().push_back(body_20_2_shape) 
body_22.GetAssets().push_back(body_20_2_level) 

# Collision shapes 
body_22.GetCollisionModel().ClearModel()
body_22.GetCollisionModel().AddSphere(mymat, 0.0045, chrono.ChVectorD(0.0987885988031631,-0.0795491716502536,-1.72553688871985E-05))
body_22.GetCollisionModel().BuildModel()
body_22.SetCollide(True)

exported_items.append(body_22)



# Rigid body part
body_23= chrono.ChBodyAuxRef()
body_23.SetName('Foot-4')
body_23.SetPos(chrono.ChVectorD(0.174786243760109,-0.00132652747527702,-0.235931546610889))
body_23.SetRot(chrono.ChQuaternionD(0.805847475772168,-0.187160021072023,0.333792953668956,-0.451844261206114))
body_23.SetMass(0.00932818757671917)
body_23.SetInertiaXX(chrono.ChVectorD(1.44259921992284e-06,1.8459776575966e-06,1.36874591235428e-06))
body_23.SetInertiaXY(chrono.ChVectorD(-4.1286803799555e-07,-3.04112106436446e-07,2.07534423694475e-07))
body_23.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0846037978523311,-0.0708768966115268,-1.72553688872246e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0.098788598803163,-0.0795491716502536,-1.7255368887234E-05)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.680657863238742,-0.191585159162366,-0.680657863238743,-0.191585159162366)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_23.GetAssets().push_back(body_20_1_level) 

# Visualization shape 
body_20_2_shape = chrono.ChObjShapeFile() 
body_20_2_shape.SetFilename(shapes_dir +'body_20_2.obj') 
body_20_2_level = chrono.ChAssetLevel() 
body_20_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0720327070443651,-0.0590889112366741,-1.72553688872618E-05)) 
body_20_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.616768955982473,0.345826625545647,-0.616768955982474,0.345826625545648)) 
body_20_2_level.GetAssets().push_back(body_20_2_shape) 
body_23.GetAssets().push_back(body_20_2_level) 

# Collision shapes 
body_23.GetCollisionModel().ClearModel()
body_23.GetCollisionModel().AddSphere(mymat, 0.0045, chrono.ChVectorD(0.098788598803163,-0.0795491716502536,-1.7255368887234E-05))
body_23.GetCollisionModel().BuildModel()
body_23.SetCollide(True)

exported_items.append(body_23)



# Rigid body part
body_24= chrono.ChBodyAuxRef()
body_24.SetName('Foot-6')
body_24.SetPos(chrono.ChVectorD(0.17493154661089,-0.00132652747527709,0.235786243760109))
body_24.SetRot(chrono.ChQuaternionD(0.805847475772169,0.187160021072024,-0.333792953668955,-0.451844261206114))
body_24.SetMass(0.00932818757671917)
body_24.SetInertiaXX(chrono.ChVectorD(1.44259921992284e-06,1.8459776575966e-06,1.36874591235428e-06))
body_24.SetInertiaXY(chrono.ChVectorD(-4.12868037995549e-07,3.04112106436447e-07,-2.07534423694476e-07))
body_24.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0846037978523311,-0.0708768966115268,-1.72553688872238e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0.098788598803163,-0.0795491716502537,-1.72553688872062E-05)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.680657863238742,-0.191585159162366,-0.680657863238743,-0.191585159162366)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_24.GetAssets().push_back(body_20_1_level) 

# Visualization shape 
body_20_2_shape = chrono.ChObjShapeFile() 
body_20_2_shape.SetFilename(shapes_dir +'body_20_2.obj') 
body_20_2_level = chrono.ChAssetLevel() 
body_20_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0720327070443652,-0.0590889112366741,-1.72553688872618E-05)) 
body_20_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.616768955982473,0.345826625545647,-0.616768955982474,0.345826625545648)) 
body_20_2_level.GetAssets().push_back(body_20_2_shape) 
body_24.GetAssets().push_back(body_20_2_level) 

# Collision shapes 
body_24.GetCollisionModel().ClearModel()
body_24.GetCollisionModel().AddSphere(mymat, 0.0045, chrono.ChVectorD(0.098788598803163,-0.0795491716502537,-1.72553688872062E-05))
body_24.GetCollisionModel().BuildModel()
body_24.SetCollide(True)

exported_items.append(body_24)



# Rigid body part
body_25= chrono.ChBodyAuxRef()
body_25.SetName('Foot-1')
body_25.SetPos(chrono.ChVectorD(-0.231967742175922,-0.00132652747527684,0.000137255368887404))
body_25.SetRot(chrono.ChQuaternionD(0.872243022401109,4.43849929013048e-16,1.84536666723215e-16,-0.489072704076376))
body_25.SetMass(0.00932818757671917)
body_25.SetInertiaXX(chrono.ChVectorD(1.31204666306433e-06,1.99496486563108e-06,1.35031126117831e-06))
body_25.SetInertiaXY(chrono.ChVectorD(-4.74093390179693e-07,4.03958864255556e-22,-6.76822199781134e-22))
body_25.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.0846037978523311,-0.0708768966115268,-1.7255368887223e-05),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_20_1_shape = chrono.ChObjShapeFile() 
body_20_1_shape.SetFilename(shapes_dir +'body_20_1.obj') 
body_20_1_level = chrono.ChAssetLevel() 
body_20_1_level.GetFrame().SetPos(chrono.ChVectorD(0.098788598803163,-0.0795491716502536,-1.72553688871954E-05)) 
body_20_1_level.GetFrame().SetRot(chrono.ChQuaternionD(0.680657863238742,-0.191585159162366,-0.680657863238743,-0.191585159162366)) 
body_20_1_level.GetAssets().push_back(body_20_1_shape) 
body_25.GetAssets().push_back(body_20_1_level) 

# Visualization shape 
body_20_2_shape = chrono.ChObjShapeFile() 
body_20_2_shape.SetFilename(shapes_dir +'body_20_2.obj') 
body_20_2_level = chrono.ChAssetLevel() 
body_20_2_level.GetFrame().SetPos(chrono.ChVectorD(0.0720327070443652,-0.0590889112366741,-1.72553688872384E-05)) 
body_20_2_level.GetFrame().SetRot(chrono.ChQuaternionD(0.616768955982473,0.345826625545647,-0.616768955982474,0.345826625545648)) 
body_20_2_level.GetAssets().push_back(body_20_2_shape) 
body_25.GetAssets().push_back(body_20_2_level) 

# Collision shapes 
body_25.GetCollisionModel().ClearModel()
body_25.GetCollisionModel().AddSphere(mymat, 0.0045, chrono.ChVectorD(0.098788598803163,-0.0795491716502536,-1.72553688871954E-05))
body_25.GetCollisionModel().BuildModel()
body_25.SetCollide(True)

exported_items.append(body_25)




# Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Hip-1/Horn-1/BHS_M2_6X8-1 ,  SW ref.type:2 (2)

link_1 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0609100775541958,0.0364,0.121910077554196)
dA = chrono.ChVectorD(8.10369721871484e-16,1,8.10369721871487e-16)
cB = chrono.ChVectorD(-0.0609100775541958,0.03943,0.121910077554196)
dB = chrono.ChVectorD(-8.34281754873893e-17,-1,-8.16720530703487e-17)
link_1.SetFlipped(True)
link_1.Initialize(body_2,body_3,False,cA,cB,dA,dB)
link_1.SetName("Concentric1")
exported_items.append(link_1)

link_2 = chrono.ChLinkMateGeneric()
link_2.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.0609100775541958,0.0364,0.121910077554196)
cB = chrono.ChVectorD(-0.0609100775541958,0.03943,0.121910077554196)
dA = chrono.ChVectorD(8.10369721871484e-16,1,8.10369721871487e-16)
dB = chrono.ChVectorD(-8.34281754873893e-17,-1,-8.16720530703487e-17)
link_2.Initialize(body_2,body_3,False,cA,cB,dA,dB)
link_2.SetName("Concentric1")
exported_items.append(link_2)


# Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-1/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Hip-1/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_3 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.0609100775541958,0.0349,0.121910077554196)
cB = chrono.ChVectorD(-0.0609100775541958,0.0349,0.121910077554196)
dA = chrono.ChVectorD(8.10369721871484e-16,1,8.10369721871487e-16)
dB = chrono.ChVectorD(-8.34281754873893e-17,-1,-8.16720530703487e-17)
link_3.Initialize(body_2,body_3,False,cA,cB,dB)
link_3.SetDistance(0)
link_3.SetName("Coincident1")
exported_items.append(link_3)

link_4 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0609100775541958,0.0349,0.121910077554196)
dA = chrono.ChVectorD(8.10369721871484e-16,1,8.10369721871487e-16)
cB = chrono.ChVectorD(-0.0609100775541958,0.0349,0.121910077554196)
dB = chrono.ChVectorD(-8.34281754873893e-17,-1,-8.16720530703487e-17)
link_4.SetFlipped(True)
link_4.Initialize(body_2,body_3,False,cA,cB,dA,dB)
link_4.SetName("Coincident1")
exported_items.append(link_4)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-2/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: Hip-2/FP04-F2-2 ,  SW ref.type:2 (2)

link_5 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.1035,0.0364000000000002,3.19735586064395e-17)
dA = chrono.ChVectorD(9.61721753139795e-16,1,6.23289211995407e-16)
cB = chrono.ChVectorD(-0.103500000000001,-0.00625999999999984,1.67600845795948e-16)
dB = chrono.ChVectorD(-4.98771180427537e-16,-1,-3.65498310565234e-16)
link_5.SetFlipped(True)
link_5.Initialize(body_2,body_14,False,cA,cB,dA,dB)
link_5.SetName("Concentric2")
exported_items.append(link_5)

link_6 = chrono.ChLinkMateGeneric()
link_6.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.1035,0.0364000000000002,3.19735586064395e-17)
cB = chrono.ChVectorD(-0.103500000000001,-0.00625999999999984,1.67600845795948e-16)
dA = chrono.ChVectorD(9.61721753139795e-16,1,6.23289211995407e-16)
dB = chrono.ChVectorD(-4.98771180427537e-16,-1,-3.65498310565234e-16)
link_6.Initialize(body_2,body_14,False,cA,cB,dA,dB)
link_6.SetName("Concentric2")
exported_items.append(link_6)


# Mate constraint: Coincident2 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-2/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: Hip-2/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_7 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.1035,0.0349000000000002,3.10386247884464e-17)
cB = chrono.ChVectorD(-0.103500000000001,0.0349000000000002,1.86619138784365e-16)
dA = chrono.ChVectorD(9.61721753139795e-16,1,6.23289211995407e-16)
dB = chrono.ChVectorD(-4.98771180427537e-16,-1,-3.65498310565234e-16)
link_7.Initialize(body_2,body_14,False,cA,cB,dB)
link_7.SetDistance(0)
link_7.SetName("Coincident2")
exported_items.append(link_7)

link_8 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.1035,0.0349000000000002,3.10386247884464e-17)
dA = chrono.ChVectorD(9.61721753139795e-16,1,6.23289211995407e-16)
cB = chrono.ChVectorD(-0.103500000000001,0.0349000000000002,1.86619138784365e-16)
dB = chrono.ChVectorD(-4.98771180427537e-16,-1,-3.65498310565234e-16)
link_8.SetFlipped(True)
link_8.Initialize(body_2,body_14,False,cA,cB,dA,dB)
link_8.SetName("Coincident2")
exported_items.append(link_8)


# Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-3/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: Hip-3/FP04-F2-2 ,  SW ref.type:2 (2)

link_9 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0609100775541958,0.0364000000000002,-0.121910077554196)
dA = chrono.ChVectorD(-4.11885256614163e-16,1,1.06946187759827e-15)
cB = chrono.ChVectorD(-0.0609100775541621,-0.00625999999999984,-0.121910077554213)
dB = chrono.ChVectorD(-8.2746487887117e-16,-1,-8.19373896182259e-16)
link_9.SetFlipped(True)
link_9.Initialize(body_2,body_7,False,cA,cB,dA,dB)
link_9.SetName("Concentric3")
exported_items.append(link_9)

link_10 = chrono.ChLinkMateGeneric()
link_10.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.0609100775541958,0.0364000000000002,-0.121910077554196)
cB = chrono.ChVectorD(-0.0609100775541621,-0.00625999999999984,-0.121910077554213)
dA = chrono.ChVectorD(-4.11885256614163e-16,1,1.06946187759827e-15)
dB = chrono.ChVectorD(-8.2746487887117e-16,-1,-8.19373896182259e-16)
link_10.Initialize(body_2,body_7,False,cA,cB,dA,dB)
link_10.SetName("Concentric3")
exported_items.append(link_10)


# Mate constraint: Coincident3 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-3/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: Hip-3/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_11 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.0609100775541958,0.0349000000000002,-0.121910077554196)
cB = chrono.ChVectorD(-0.060910077554162,0.0349000000000002,-0.121910077554213)
dA = chrono.ChVectorD(-4.11885256614163e-16,1,1.06946187759827e-15)
dB = chrono.ChVectorD(-8.2746487887117e-16,-1,-8.19373896182259e-16)
link_11.Initialize(body_2,body_7,False,cA,cB,dB)
link_11.SetDistance(0)
link_11.SetName("Coincident3")
exported_items.append(link_11)

link_12 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0609100775541958,0.0349000000000002,-0.121910077554196)
dA = chrono.ChVectorD(-4.11885256614163e-16,1,1.06946187759827e-15)
cB = chrono.ChVectorD(-0.060910077554162,0.0349000000000002,-0.121910077554213)
dB = chrono.ChVectorD(-8.2746487887117e-16,-1,-8.19373896182259e-16)
link_12.SetFlipped(True)
link_12.Initialize(body_2,body_7,False,cA,cB,dA,dB)
link_12.SetName("Coincident3")
exported_items.append(link_12)


# Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-9/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: Hip-4/FP04-F2-2 ,  SW ref.type:2 (2)

link_13 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.0609100775541958,0.0364000000000001,0.121910077554196)
dA = chrono.ChVectorD(3.57263954205419e-16,1,1.08892636999829e-15)
cB = chrono.ChVectorD(0.0609100775541958,-0.00625999999999991,0.121910077554196)
dB = chrono.ChVectorD(-8.67534831605032e-16,-1,-8.75625814294388e-16)
link_13.SetFlipped(True)
link_13.Initialize(body_2,body_19,False,cA,cB,dA,dB)
link_13.SetName("Concentric4")
exported_items.append(link_13)

link_14 = chrono.ChLinkMateGeneric()
link_14.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.0609100775541958,0.0364000000000001,0.121910077554196)
cB = chrono.ChVectorD(0.0609100775541958,-0.00625999999999991,0.121910077554196)
dA = chrono.ChVectorD(3.57263954205419e-16,1,1.08892636999829e-15)
dB = chrono.ChVectorD(-8.67534831605032e-16,-1,-8.75625814294388e-16)
link_14.Initialize(body_2,body_19,False,cA,cB,dA,dB)
link_14.SetName("Concentric4")
exported_items.append(link_14)


# Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-9/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: Hip-4/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_15 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.0609100775541958,0.0349000000000001,0.121910077554196)
cB = chrono.ChVectorD(0.0609100775541958,0.0349000000000001,0.121910077554196)
dA = chrono.ChVectorD(3.57263954205419e-16,1,1.08892636999829e-15)
dB = chrono.ChVectorD(-8.67534831605032e-16,-1,-8.75625814294388e-16)
link_15.Initialize(body_2,body_19,False,cA,cB,dB)
link_15.SetDistance(0)
link_15.SetName("Coincident4")
exported_items.append(link_15)

link_16 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.0609100775541958,0.0349000000000001,0.121910077554196)
dA = chrono.ChVectorD(3.57263954205419e-16,1,1.08892636999829e-15)
cB = chrono.ChVectorD(0.0609100775541958,0.0349000000000001,0.121910077554196)
dB = chrono.ChVectorD(-8.67534831605032e-16,-1,-8.75625814294388e-16)
link_16.SetFlipped(True)
link_16.Initialize(body_2,body_19,False,cA,cB,dA,dB)
link_16.SetName("Coincident4")
exported_items.append(link_16)


# Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-8/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Hip-5/FP04-F2-2 ,  SW ref.type:2 (2)

link_17 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.1035,0.0364,2.11193216868351e-17)
dA = chrono.ChVectorD(-3.01339605878215e-31,1,-1.14603585120717e-15)
cB = chrono.ChVectorD(0.1035,-0.00626,1.28484984349597e-18)
dB = chrono.ChVectorD(-1.28534861418875e-15,-1,-5.36848812845108e-16)
link_17.SetFlipped(True)
link_17.Initialize(body_2,body_8,False,cA,cB,dA,dB)
link_17.SetName("Concentric5")
exported_items.append(link_17)

link_18 = chrono.ChLinkMateGeneric()
link_18.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.1035,0.0364,2.11193216868351e-17)
cB = chrono.ChVectorD(0.1035,-0.00626,1.28484984349597e-18)
dA = chrono.ChVectorD(-3.01339605878215e-31,1,-1.14603585120717e-15)
dB = chrono.ChVectorD(-1.28534861418875e-15,-1,-5.36848812845108e-16)
link_18.Initialize(body_2,body_8,False,cA,cB,dA,dB)
link_18.SetName("Concentric5")
exported_items.append(link_18)


# Mate constraint: Coincident5 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-8/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Hip-5/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_19 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.1035,0.0349,2.28383754636458e-17)
cB = chrono.ChVectorD(0.1035,0.0349,3.36207718938459e-17)
dA = chrono.ChVectorD(-3.01339605878215e-31,1,-1.14603585120717e-15)
dB = chrono.ChVectorD(-1.28534861418875e-15,-1,-5.36848812845108e-16)
link_19.Initialize(body_2,body_8,False,cA,cB,dB)
link_19.SetDistance(0)
link_19.SetName("Coincident5")
exported_items.append(link_19)

link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.1035,0.0349,2.28383754636458e-17)
dA = chrono.ChVectorD(-3.01339605878215e-31,1,-1.14603585120717e-15)
cB = chrono.ChVectorD(0.1035,0.0349,3.36207718938459e-17)
dB = chrono.ChVectorD(-1.28534861418875e-15,-1,-5.36848812845108e-16)
link_20.SetFlipped(True)
link_20.Initialize(body_2,body_8,False,cA,cB,dA,dB)
link_20.SetName("Coincident5")
exported_items.append(link_20)


# Mate constraint: Concentric6 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-7/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: Hip-6/FP04-F2-2 ,  SW ref.type:2 (2)

link_21 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.0609100775541958,0.0364000000000001,-0.121910077554196)
dA = chrono.ChVectorD(-4.7325379075631e-16,1,1.04375716610089e-15)
cB = chrono.ChVectorD(0.0609100775541958,-0.00625999999999984,-0.121910077554196)
dB = chrono.ChVectorD(-8.10369721871439e-16,-1,-8.10369721871482e-16)
link_21.SetFlipped(True)
link_21.Initialize(body_2,body_15,False,cA,cB,dA,dB)
link_21.SetName("Concentric6")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateGeneric()
link_22.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.0609100775541958,0.0364000000000001,-0.121910077554196)
cB = chrono.ChVectorD(0.0609100775541958,-0.00625999999999984,-0.121910077554196)
dA = chrono.ChVectorD(-4.7325379075631e-16,1,1.04375716610089e-15)
dB = chrono.ChVectorD(-8.10369721871439e-16,-1,-8.10369721871482e-16)
link_22.Initialize(body_2,body_15,False,cA,cB,dA,dB)
link_22.SetName("Concentric6")
exported_items.append(link_22)


# Mate constraint: Coincident6 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-7/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: Hip-6/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_23 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.0609100775541958,0.0349000000000001,-0.121910077554196)
cB = chrono.ChVectorD(0.0609100775541958,0.0349000000000002,-0.121910077554196)
dA = chrono.ChVectorD(-4.7325379075631e-16,1,1.04375716610089e-15)
dB = chrono.ChVectorD(-8.10369721871439e-16,-1,-8.10369721871482e-16)
link_23.Initialize(body_2,body_15,False,cA,cB,dB)
link_23.SetDistance(0)
link_23.SetName("Coincident6")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.0609100775541958,0.0349000000000001,-0.121910077554196)
dA = chrono.ChVectorD(-4.7325379075631e-16,1,1.04375716610089e-15)
cB = chrono.ChVectorD(0.0609100775541958,0.0349000000000002,-0.121910077554196)
dB = chrono.ChVectorD(-8.10369721871439e-16,-1,-8.10369721871482e-16)
link_24.SetFlipped(True)
link_24.Initialize(body_2,body_15,False,cA,cB,dA,dB)
link_24.SetName("Coincident6")
exported_items.append(link_24)


# Mate constraint: Concentric7 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: Femur-1/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Hip-1/Horn-4/BHS_M2_6X8-1 ,  SW ref.type:2 (2)

link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0849799923857858,0.01774,0.171379267966007)
dA = chrono.ChVectorD(0.707106781186547,-4.85722573273506e-17,0.707106781186548)
cB = chrono.ChVectorD(-0.08234248409196,0.01774,0.174016776259833)
dB = chrono.ChVectorD(-0.707106781186547,6.12323399573676e-17,-0.707106781186548)
link_25.SetFlipped(True)
link_25.Initialize(body_4,body_3,False,cA,cB,dA,dB)
link_25.SetName("Concentric7")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.0849799923857858,0.01774,0.171379267966007)
cB = chrono.ChVectorD(-0.08234248409196,0.01774,0.174016776259833)
dA = chrono.ChVectorD(0.707106781186547,-4.85722573273506e-17,0.707106781186548)
dB = chrono.ChVectorD(-0.707106781186547,6.12323399573676e-17,-0.707106781186548)
link_26.Initialize(body_4,body_3,False,cA,cB,dA,dB)
link_26.SetName("Concentric7")
exported_items.append(link_26)


# Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: Hip-1/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Femur-1/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)

link_27 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.0849799923857859,0.01774,0.171379267966007)
cB = chrono.ChVectorD(-0.0849799923857858,0.01774,0.171379267966007)
dA = chrono.ChVectorD(-0.707106781186547,6.12323399573676e-17,-0.707106781186548)
dB = chrono.ChVectorD(0.707106781186547,-4.85722573273506e-17,0.707106781186548)
link_27.Initialize(body_3,body_4,False,cA,cB,dB)
link_27.SetDistance(0)
link_27.SetName("Coincident7")
exported_items.append(link_27)

link_28 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.0849799923857859,0.01774,0.171379267966007)
dA = chrono.ChVectorD(-0.707106781186547,6.12323399573676e-17,-0.707106781186548)
cB = chrono.ChVectorD(-0.0849799923857858,0.01774,0.171379267966007)
dB = chrono.ChVectorD(0.707106781186547,-4.85722573273506e-17,0.707106781186548)
link_28.SetFlipped(True)
link_28.Initialize(body_3,body_4,False,cA,cB,dA,dB)
link_28.SetName("Coincident7")
exported_items.append(link_28)


# Mate constraint: Concentric8 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: Femur-2/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: Hip-2/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_29 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.1555,0.0177400000000002,0.0179600000000003)
dA = chrono.ChVectorD(2.22044604925031e-16,-4.44089209850063e-16,1)
cB = chrono.ChVectorD(-0.155500000000001,0.0177400000000002,0.0143000000000002)
dB = chrono.ChVectorD(0,0,-1)
link_29.SetFlipped(True)
link_29.Initialize(body_6,body_14,False,cA,cB,dA,dB)
link_29.SetName("Concentric8")
exported_items.append(link_29)

link_30 = chrono.ChLinkMateGeneric()
link_30.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.1555,0.0177400000000002,0.0179600000000003)
cB = chrono.ChVectorD(-0.155500000000001,0.0177400000000002,0.0143000000000002)
dA = chrono.ChVectorD(2.22044604925031e-16,-4.44089209850063e-16,1)
dB = chrono.ChVectorD(0,0,-1)
link_30.Initialize(body_6,body_14,False,cA,cB,dA,dB)
link_30.SetName("Concentric8")
exported_items.append(link_30)


# Mate constraint: Coincident8 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: Femur-2/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: Hip-2/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_31 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.1555,0.0177400000000002,0.0179600000000003)
cB = chrono.ChVectorD(-0.155500000000001,0.0177400000000002,0.0179600000000002)
dA = chrono.ChVectorD(2.22044604925031e-16,-4.44089209850063e-16,1)
dB = chrono.ChVectorD(0,0,-1)
link_31.Initialize(body_6,body_14,False,cA,cB,dB)
link_31.SetDistance(0)
link_31.SetName("Coincident8")
exported_items.append(link_31)

link_32 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.1555,0.0177400000000002,0.0179600000000003)
dA = chrono.ChVectorD(2.22044604925031e-16,-4.44089209850063e-16,1)
cB = chrono.ChVectorD(-0.155500000000001,0.0177400000000002,0.0179600000000002)
dB = chrono.ChVectorD(0,0,-1)
link_32.SetFlipped(True)
link_32.Initialize(body_6,body_14,False,cA,cB,dA,dB)
link_32.SetName("Coincident8")
exported_items.append(link_32)


# Mate constraint: Concentric9 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_17 , SW name: Femur-3/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: Hip-3/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_33 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.110379267965966,0.0177400000000002,-0.145979992385816)
dA = chrono.ChVectorD(-0.707106781186741,6.12323399573676e-17,0.707106781186354)
cB = chrono.ChVectorD(-0.107791257146823,0.0177400000000002,-0.148568003204958)
dB = chrono.ChVectorD(0.707106781186741,-6.12323399573681e-17,-0.707106781186354)
link_33.SetFlipped(True)
link_33.Initialize(body_17,body_7,False,cA,cB,dA,dB)
link_33.SetName("Concentric9")
exported_items.append(link_33)

link_34 = chrono.ChLinkMateGeneric()
link_34.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.110379267965966,0.0177400000000002,-0.145979992385816)
cB = chrono.ChVectorD(-0.107791257146823,0.0177400000000002,-0.148568003204958)
dA = chrono.ChVectorD(-0.707106781186741,6.12323399573676e-17,0.707106781186354)
dB = chrono.ChVectorD(0.707106781186741,-6.12323399573681e-17,-0.707106781186354)
link_34.Initialize(body_17,body_7,False,cA,cB,dA,dB)
link_34.SetName("Concentric9")
exported_items.append(link_34)


# Mate constraint: Coincident9 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_17 , SW name: Femur-3/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: Hip-3/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_35 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.110379267965966,0.0177400000000002,-0.145979992385816)
cB = chrono.ChVectorD(-0.110379267965966,0.0177400000000002,-0.145979992385816)
dA = chrono.ChVectorD(-0.707106781186741,6.12323399573676e-17,0.707106781186354)
dB = chrono.ChVectorD(0.707106781186741,-6.12323399573681e-17,-0.707106781186354)
link_35.Initialize(body_17,body_7,False,cA,cB,dB)
link_35.SetDistance(0)
link_35.SetName("Coincident9")
exported_items.append(link_35)

link_36 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.110379267965966,0.0177400000000002,-0.145979992385816)
dA = chrono.ChVectorD(-0.707106781186741,6.12323399573676e-17,0.707106781186354)
cB = chrono.ChVectorD(-0.110379267965966,0.0177400000000002,-0.145979992385816)
dB = chrono.ChVectorD(0.707106781186741,-6.12323399573681e-17,-0.707106781186354)
link_36.SetFlipped(True)
link_36.Initialize(body_17,body_7,False,cA,cB,dA,dB)
link_36.SetName("Coincident9")
exported_items.append(link_36)


# Mate constraint: Concentric10 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_16 , SW name: Femur-4/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: Hip-4/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_37 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.110379267966007,0.01774,0.145979992385786)
dA = chrono.ChVectorD(0.707106781186546,6.12323399573677e-17,-0.707106781186549)
cB = chrono.ChVectorD(0.107791257146864,0.01774,0.148568003204929)
dB = chrono.ChVectorD(-0.707106781186546,-6.12323399573679e-17,0.70710678118655)
link_37.SetFlipped(True)
link_37.Initialize(body_16,body_19,False,cA,cB,dA,dB)
link_37.SetName("Concentric10")
exported_items.append(link_37)

link_38 = chrono.ChLinkMateGeneric()
link_38.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.110379267966007,0.01774,0.145979992385786)
cB = chrono.ChVectorD(0.107791257146864,0.01774,0.148568003204929)
dA = chrono.ChVectorD(0.707106781186546,6.12323399573677e-17,-0.707106781186549)
dB = chrono.ChVectorD(-0.707106781186546,-6.12323399573679e-17,0.70710678118655)
link_38.Initialize(body_16,body_19,False,cA,cB,dA,dB)
link_38.SetName("Concentric10")
exported_items.append(link_38)


# Mate constraint: Coincident10 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_16 , SW name: Femur-4/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: Hip-4/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_39 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.110379267966007,0.01774,0.145979992385786)
cB = chrono.ChVectorD(0.110379267966007,0.01774,0.145979992385786)
dA = chrono.ChVectorD(0.707106781186546,6.12323399573677e-17,-0.707106781186549)
dB = chrono.ChVectorD(-0.707106781186546,-6.12323399573679e-17,0.70710678118655)
link_39.Initialize(body_16,body_19,False,cA,cB,dB)
link_39.SetDistance(0)
link_39.SetName("Coincident10")
exported_items.append(link_39)

link_40 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.110379267966007,0.01774,0.145979992385786)
dA = chrono.ChVectorD(0.707106781186546,6.12323399573677e-17,-0.707106781186549)
cB = chrono.ChVectorD(0.110379267966007,0.01774,0.145979992385786)
dB = chrono.ChVectorD(-0.707106781186546,-6.12323399573679e-17,0.70710678118655)
link_40.SetFlipped(True)
link_40.Initialize(body_16,body_19,False,cA,cB,dA,dB)
link_40.SetName("Coincident10")
exported_items.append(link_40)


# Mate constraint: Concentric11 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: Femur-5/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Hip-5/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_41 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.1555,0.0177399999999999,-0.01796)
dA = chrono.ChVectorD(1.49201433997704e-16,5.85884629837978e-16,-1)
cB = chrono.ChVectorD(0.1555,0.0177399999999999,-0.0143)
dB = chrono.ChVectorD(-2.77555756156289e-17,-5.92359964076365e-16,1)
link_41.SetFlipped(True)
link_41.Initialize(body_10,body_8,False,cA,cB,dA,dB)
link_41.SetName("Concentric11")
exported_items.append(link_41)

link_42 = chrono.ChLinkMateGeneric()
link_42.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.1555,0.0177399999999999,-0.01796)
cB = chrono.ChVectorD(0.1555,0.0177399999999999,-0.0143)
dA = chrono.ChVectorD(1.49201433997704e-16,5.85884629837978e-16,-1)
dB = chrono.ChVectorD(-2.77555756156289e-17,-5.92359964076365e-16,1)
link_42.Initialize(body_10,body_8,False,cA,cB,dA,dB)
link_42.SetName("Concentric11")
exported_items.append(link_42)


# Mate constraint: Coincident11 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: Femur-5/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Hip-5/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_43 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.1555,0.0177399999999999,-0.01796)
cB = chrono.ChVectorD(0.1555,0.0177399999999999,-0.01796)
dA = chrono.ChVectorD(1.49201433997704e-16,5.85884629837978e-16,-1)
dB = chrono.ChVectorD(-2.77555756156289e-17,-5.92359964076365e-16,1)
link_43.Initialize(body_10,body_8,False,cA,cB,dB)
link_43.SetDistance(0)
link_43.SetName("Coincident11")
exported_items.append(link_43)

link_44 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.1555,0.0177399999999999,-0.01796)
dA = chrono.ChVectorD(1.49201433997704e-16,5.85884629837978e-16,-1)
cB = chrono.ChVectorD(0.1555,0.0177399999999999,-0.01796)
dB = chrono.ChVectorD(-2.77555756156289e-17,-5.92359964076365e-16,1)
link_44.SetFlipped(True)
link_44.Initialize(body_10,body_8,False,cA,cB,dA,dB)
link_44.SetName("Coincident11")
exported_items.append(link_44)


# Mate constraint: Concentric12 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_18 , SW name: Femur-6/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: Hip-6/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_45 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.084979992385786,0.0177400000000002,-0.171379267966007)
dA = chrono.ChVectorD(-0.707106781186547,1.14491749414469e-15,-0.707106781186548)
cB = chrono.ChVectorD(0.0875680032049287,0.0177400000000002,-0.168791257146864)
dB = chrono.ChVectorD(0.707106781186547,-1.20154700243839e-15,0.707106781186548)
link_45.SetFlipped(True)
link_45.Initialize(body_18,body_15,False,cA,cB,dA,dB)
link_45.SetName("Concentric12")
exported_items.append(link_45)

link_46 = chrono.ChLinkMateGeneric()
link_46.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.084979992385786,0.0177400000000002,-0.171379267966007)
cB = chrono.ChVectorD(0.0875680032049287,0.0177400000000002,-0.168791257146864)
dA = chrono.ChVectorD(-0.707106781186547,1.14491749414469e-15,-0.707106781186548)
dB = chrono.ChVectorD(0.707106781186547,-1.20154700243839e-15,0.707106781186548)
link_46.Initialize(body_18,body_15,False,cA,cB,dA,dB)
link_46.SetName("Concentric12")
exported_items.append(link_46)


# Mate constraint: Coincident12 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_18 , SW name: Femur-6/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: Hip-6/Horn-4/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_47 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.084979992385786,0.0177400000000002,-0.171379267966007)
cB = chrono.ChVectorD(0.0849799923857859,0.0177400000000002,-0.171379267966007)
dA = chrono.ChVectorD(-0.707106781186547,1.14491749414469e-15,-0.707106781186548)
dB = chrono.ChVectorD(0.707106781186547,-1.20154700243839e-15,0.707106781186548)
link_47.Initialize(body_18,body_15,False,cA,cB,dB)
link_47.SetDistance(0)
link_47.SetName("Coincident12")
exported_items.append(link_47)

link_48 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.084979992385786,0.0177400000000002,-0.171379267966007)
dA = chrono.ChVectorD(-0.707106781186547,1.14491749414469e-15,-0.707106781186548)
cB = chrono.ChVectorD(0.0849799923857859,0.0177400000000002,-0.171379267966007)
dB = chrono.ChVectorD(0.707106781186547,-1.20154700243839e-15,0.707106781186548)
link_48.SetFlipped(True)
link_48.Initialize(body_18,body_15,False,cA,cB,dA,dB)
link_48.SetName("Coincident12")
exported_items.append(link_48)


# Mate constraint: Concentric13 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Tibia-1/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Femur-1/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_49 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.130496136579496,0.00315104747200021,0.216980264973459)
dA = chrono.ChVectorD(0.707106781186547,-2.77555756156289e-17,0.707106781186548)
cB = chrono.ChVectorD(-0.131061822004452,0.00315104747200021,0.216414579548503)
dB = chrono.ChVectorD(-0.707106781186547,8.5868812060852e-17,-0.707106781186548)
link_49.SetFlipped(True)
link_49.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_49.SetName("Concentric13")
exported_items.append(link_49)

link_50 = chrono.ChLinkMateGeneric()
link_50.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.130496136579496,0.00315104747200021,0.216980264973459)
cB = chrono.ChVectorD(-0.131061822004452,0.00315104747200021,0.216414579548503)
dA = chrono.ChVectorD(0.707106781186547,-2.77555756156289e-17,0.707106781186548)
dB = chrono.ChVectorD(-0.707106781186547,8.5868812060852e-17,-0.707106781186548)
link_50.Initialize(body_1,body_4,False,cA,cB,dA,dB)
link_50.SetName("Concentric13")
exported_items.append(link_50)


# Mate constraint: Coincident13 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: Femur-1/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: Tibia-1/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)

link_51 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.130496136579496,0.00315104747200021,0.216980264973459)
cB = chrono.ChVectorD(-0.130496136579496,0.00315104747200021,0.216980264973459)
dA = chrono.ChVectorD(-0.707106781186547,8.5868812060852e-17,-0.707106781186548)
dB = chrono.ChVectorD(0.707106781186547,-2.77555756156289e-17,0.707106781186548)
link_51.Initialize(body_4,body_1,False,cA,cB,dB)
link_51.SetDistance(0)
link_51.SetName("Coincident13")
exported_items.append(link_51)

link_52 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.130496136579496,0.00315104747200021,0.216980264973459)
dA = chrono.ChVectorD(-0.707106781186547,8.5868812060852e-17,-0.707106781186548)
cB = chrono.ChVectorD(-0.130496136579496,0.00315104747200021,0.216980264973459)
dB = chrono.ChVectorD(0.707106781186547,-2.77555756156289e-17,0.707106781186548)
link_52.SetFlipped(True)
link_52.Initialize(body_4,body_1,False,cA,cB,dA,dB)
link_52.SetName("Coincident13")
exported_items.append(link_52)


# Mate constraint: Concentric14 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: Tibia-2/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: Femur-2/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_53 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.219929548425674,0.00315104747200037,0.0180200000000003)
dA = chrono.ChVectorD(-1.12228130120072e-16,-9.54793700340844e-16,1)
cB = chrono.ChVectorD(-0.219929548425675,0.00315104747200036,0.0143600000000003)
dB = chrono.ChVectorD(-2.80519432283827e-16,4.25502460569834e-16,-1)
link_53.SetFlipped(True)
link_53.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_53.SetName("Concentric14")
exported_items.append(link_53)

link_54 = chrono.ChLinkMateGeneric()
link_54.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.219929548425674,0.00315104747200037,0.0180200000000003)
cB = chrono.ChVectorD(-0.219929548425675,0.00315104747200036,0.0143600000000003)
dA = chrono.ChVectorD(-1.12228130120072e-16,-9.54793700340844e-16,1)
dB = chrono.ChVectorD(-2.80519432283827e-16,4.25502460569834e-16,-1)
link_54.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_54.SetName("Concentric14")
exported_items.append(link_54)


# Mate constraint: Coincident14 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: Tibia-2/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: Femur-2/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_55 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.219929548425674,0.00315104747200037,0.0180200000000003)
cB = chrono.ChVectorD(-0.219929548425675,0.00315104747200036,0.0180200000000003)
dA = chrono.ChVectorD(-1.12228130120072e-16,-9.54793700340844e-16,1)
dB = chrono.ChVectorD(-2.80519432283827e-16,4.25502460569834e-16,-1)
link_55.Initialize(body_5,body_6,False,cA,cB,dB)
link_55.SetDistance(0)
link_55.SetName("Coincident14")
exported_items.append(link_55)

link_56 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.219929548425674,0.00315104747200037,0.0180200000000003)
dA = chrono.ChVectorD(-1.12228130120072e-16,-9.54793700340844e-16,1)
cB = chrono.ChVectorD(-0.219929548425675,0.00315104747200036,0.0180200000000003)
dB = chrono.ChVectorD(-2.80519432283827e-16,4.25502460569834e-16,-1)
link_56.SetFlipped(True)
link_56.Initialize(body_5,body_6,False,cA,cB,dA,dB)
link_56.SetName("Coincident14")
exported_items.append(link_56)


# Mate constraint: Concentric15 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: Tibia-3/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: Femur-3/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_57 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.155980264973407,0.00315104747200041,-0.191496136579539)
dA = chrono.ChVectorD(-0.707106781186741,4.9960036108132e-16,0.707106781186354)
cB = chrono.ChVectorD(-0.153392254154263,0.00315104747200043,-0.194084147398681)
dB = chrono.ChVectorD(0.707106781186741,-7.98190892375965e-17,-0.707106781186354)
link_57.SetFlipped(True)
link_57.Initialize(body_11,body_17,False,cA,cB,dA,dB)
link_57.SetName("Concentric15")
exported_items.append(link_57)

link_58 = chrono.ChLinkMateGeneric()
link_58.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(-0.155980264973407,0.00315104747200041,-0.191496136579539)
cB = chrono.ChVectorD(-0.153392254154263,0.00315104747200043,-0.194084147398681)
dA = chrono.ChVectorD(-0.707106781186741,4.9960036108132e-16,0.707106781186354)
dB = chrono.ChVectorD(0.707106781186741,-7.98190892375965e-17,-0.707106781186354)
link_58.Initialize(body_11,body_17,False,cA,cB,dA,dB)
link_58.SetName("Concentric15")
exported_items.append(link_58)


# Mate constraint: Coincident15 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: Tibia-3/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: Femur-3/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_59 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.155980264973407,0.00315104747200041,-0.191496136579539)
cB = chrono.ChVectorD(-0.155980264973407,0.00315104747200043,-0.191496136579539)
dA = chrono.ChVectorD(-0.707106781186741,4.9960036108132e-16,0.707106781186354)
dB = chrono.ChVectorD(0.707106781186741,-7.98190892375965e-17,-0.707106781186354)
link_59.Initialize(body_11,body_17,False,cA,cB,dB)
link_59.SetDistance(0)
link_59.SetName("Coincident15")
exported_items.append(link_59)

link_60 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.155980264973407,0.00315104747200041,-0.191496136579539)
dA = chrono.ChVectorD(-0.707106781186741,4.9960036108132e-16,0.707106781186354)
cB = chrono.ChVectorD(-0.155980264973407,0.00315104747200043,-0.191496136579539)
dB = chrono.ChVectorD(0.707106781186741,-7.98190892375965e-17,-0.707106781186354)
link_60.SetFlipped(True)
link_60.Initialize(body_11,body_17,False,cA,cB,dA,dB)
link_60.SetName("Coincident15")
exported_items.append(link_60)


# Mate constraint: Concentric16 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Tibia-4/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: Femur-4/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_61 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.15598026497346,0.00315104747200022,0.191496136579496)
dA = chrono.ChVectorD(0.707106781186545,5.55111512312578e-17,-0.70710678118655)
cB = chrono.ChVectorD(0.153392254154317,0.00315104747200022,0.194084147398639)
dB = chrono.ChVectorD(-0.707106781186546,-8.52182907573607e-17,0.707106781186549)
link_61.SetFlipped(True)
link_61.Initialize(body_9,body_16,False,cA,cB,dA,dB)
link_61.SetName("Concentric16")
exported_items.append(link_61)

link_62 = chrono.ChLinkMateGeneric()
link_62.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.15598026497346,0.00315104747200022,0.191496136579496)
cB = chrono.ChVectorD(0.153392254154317,0.00315104747200022,0.194084147398639)
dA = chrono.ChVectorD(0.707106781186545,5.55111512312578e-17,-0.70710678118655)
dB = chrono.ChVectorD(-0.707106781186546,-8.52182907573607e-17,0.707106781186549)
link_62.Initialize(body_9,body_16,False,cA,cB,dA,dB)
link_62.SetName("Concentric16")
exported_items.append(link_62)


# Mate constraint: Coincident16 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Tibia-4/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: Femur-4/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_63 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.15598026497346,0.00315104747200022,0.191496136579496)
cB = chrono.ChVectorD(0.15598026497346,0.00315104747200022,0.191496136579496)
dA = chrono.ChVectorD(0.707106781186545,5.55111512312578e-17,-0.70710678118655)
dB = chrono.ChVectorD(-0.707106781186546,-8.52182907573607e-17,0.707106781186549)
link_63.Initialize(body_9,body_16,False,cA,cB,dB)
link_63.SetDistance(0)
link_63.SetName("Coincident16")
exported_items.append(link_63)

link_64 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.15598026497346,0.00315104747200022,0.191496136579496)
dA = chrono.ChVectorD(0.707106781186545,5.55111512312578e-17,-0.70710678118655)
cB = chrono.ChVectorD(0.15598026497346,0.00315104747200022,0.191496136579496)
dB = chrono.ChVectorD(-0.707106781186546,-8.52182907573607e-17,0.707106781186549)
link_64.SetFlipped(True)
link_64.Initialize(body_9,body_16,False,cA,cB,dA,dB)
link_64.SetName("Coincident16")
exported_items.append(link_64)


# Mate constraint: Concentric17 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_12 , SW name: Tibia-5/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: Femur-5/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_65 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.219929548425675,0.00315104747200014,-0.01802)
dA = chrono.ChVectorD(3.52740885930265e-16,6.30171164314264e-16,-1)
cB = chrono.ChVectorD(0.219929548425675,0.00315104747200013,-0.01436)
dB = chrono.ChVectorD(-9.0726606638909e-17,-6.04471379118207e-16,1)
link_65.SetFlipped(True)
link_65.Initialize(body_12,body_10,False,cA,cB,dA,dB)
link_65.SetName("Concentric17")
exported_items.append(link_65)

link_66 = chrono.ChLinkMateGeneric()
link_66.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.219929548425675,0.00315104747200014,-0.01802)
cB = chrono.ChVectorD(0.219929548425675,0.00315104747200013,-0.01436)
dA = chrono.ChVectorD(3.52740885930265e-16,6.30171164314264e-16,-1)
dB = chrono.ChVectorD(-9.0726606638909e-17,-6.04471379118207e-16,1)
link_66.Initialize(body_12,body_10,False,cA,cB,dA,dB)
link_66.SetName("Concentric17")
exported_items.append(link_66)


# Mate constraint: Coincident17 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_12 , SW name: Tibia-5/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: Femur-5/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_67 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.219929548425675,0.00315104747200014,-0.01802)
cB = chrono.ChVectorD(0.219929548425675,0.00315104747200014,-0.01802)
dA = chrono.ChVectorD(3.52740885930265e-16,6.30171164314264e-16,-1)
dB = chrono.ChVectorD(-9.0726606638909e-17,-6.04471379118207e-16,1)
link_67.Initialize(body_12,body_10,False,cA,cB,dB)
link_67.SetDistance(0)
link_67.SetName("Coincident17")
exported_items.append(link_67)

link_68 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.219929548425675,0.00315104747200014,-0.01802)
dA = chrono.ChVectorD(3.52740885930265e-16,6.30171164314264e-16,-1)
cB = chrono.ChVectorD(0.219929548425675,0.00315104747200014,-0.01802)
dB = chrono.ChVectorD(-9.0726606638909e-17,-6.04471379118207e-16,1)
link_68.SetFlipped(True)
link_68.Initialize(body_12,body_10,False,cA,cB,dA,dB)
link_68.SetName("Coincident17")
exported_items.append(link_68)


# Mate constraint: Concentric18 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: Tibia-6/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: Femur-6/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_69 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.130496136579496,0.00315104747200035,-0.216980264973459)
dA = chrono.ChVectorD(-0.707106781186547,9.15933995315754e-16,-0.707106781186548)
cB = chrono.ChVectorD(0.133084147398639,0.00315104747200035,-0.214392254154317)
dB = chrono.ChVectorD(0.707106781186547,-1.15597635630405e-15,0.707106781186548)
link_69.SetFlipped(True)
link_69.Initialize(body_13,body_18,False,cA,cB,dA,dB)
link_69.SetName("Concentric18")
exported_items.append(link_69)

link_70 = chrono.ChLinkMateGeneric()
link_70.SetConstrainedCoords(False, True, True, False, False, False)
cA = chrono.ChVectorD(0.130496136579496,0.00315104747200035,-0.216980264973459)
cB = chrono.ChVectorD(0.133084147398639,0.00315104747200035,-0.214392254154317)
dA = chrono.ChVectorD(-0.707106781186547,9.15933995315754e-16,-0.707106781186548)
dB = chrono.ChVectorD(0.707106781186547,-1.15597635630405e-15,0.707106781186548)
link_70.Initialize(body_13,body_18,False,cA,cB,dA,dB)
link_70.SetName("Concentric18")
exported_items.append(link_70)


# Mate constraint: Coincident18 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: Tibia-6/AX-18F-1/ASSY_GEAR-WHEEL___DUMMY-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: Femur-6/Horn-1/DC04_B01_HORN2_DUMMY-1 ,  SW ref.type:2 (2)

link_71 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.130496136579496,0.00315104747200035,-0.216980264973459)
cB = chrono.ChVectorD(0.130496136579496,0.00315104747200035,-0.216980264973459)
dA = chrono.ChVectorD(-0.707106781186547,9.15933995315754e-16,-0.707106781186548)
dB = chrono.ChVectorD(0.707106781186547,-1.15597635630405e-15,0.707106781186548)
link_71.Initialize(body_13,body_18,False,cA,cB,dB)
link_71.SetDistance(0)
link_71.SetName("Coincident18")
exported_items.append(link_71)

link_72 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.130496136579496,0.00315104747200035,-0.216980264973459)
dA = chrono.ChVectorD(-0.707106781186547,9.15933995315754e-16,-0.707106781186548)
cB = chrono.ChVectorD(0.130496136579496,0.00315104747200035,-0.216980264973459)
dB = chrono.ChVectorD(0.707106781186547,-1.15597635630405e-15,0.707106781186548)
link_72.SetFlipped(True)
link_72.Initialize(body_13,body_18,False,cA,cB,dA,dB)
link_72.SetName("Coincident18")
exported_items.append(link_72)


# Mate constraint: Angle1 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: Femur-1/AX-18F-1/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle2 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-1/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: Hip-1/FP04-F2-2 ,  SW ref.type:2 (2)


# Mate constraint: Angle4 [MatePlanarAngleDim] type:6 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_21 , SW name: Foot-2/Foot-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle5 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_14 , SW name: Hip-2/FP04-F2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle6 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: Femur-2/AX-18F-1/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle8 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: Hip-3/FP04-F2-2 ,  SW ref.type:2 (2)


# Mate constraint: Angle9 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: Femur-3/AX-18F-1/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle10 [MatePlanarAngleDim] type:6 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_25 , SW name: Foot-1/Foot-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle14 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_19 , SW name: Hip-4/FP04-F2-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle11 [MatePlanarAngleDim] type:6 align:0 flip:True
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: Femur-4/AX-18F-1/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle12 [MatePlanarAngleDim] type:6 align:1 flip:False
#   Entity 0: C::E name: body_20 , SW name: Foot-3/Foot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle13 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_8 , SW name: Hip-5/FP04-F2-2 ,  SW ref.type:2 (2)


# Mate constraint: Angle15 [MatePlanarAngleDim] type:6 align:1 flip:True
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: Femur-5/AX-18F-1/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle16 [MatePlanarAngleDim] type:6 align:1 flip:False
#   Entity 0: C::E name: body_23 , SW name: Foot-4/Foot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle17 [MatePlanarAngleDim] type:6 align:1 flip:True
#   Entity 0: C::E name: body_2 , SW name: Body-1/AX-18F-7/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_15 , SW name: Hip-6/FP04-F2-2 ,  SW ref.type:2 (2)


# Mate constraint: Angle18 [MatePlanarAngleDim] type:6 align:0 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: Femur-6/AX-18F-1/ASSY_DUMMY___-1 ,  SW ref.type:2 (2)


# Mate constraint: Angle19 [MatePlanarAngleDim] type:6 align:1 flip:True
#   Entity 0: C::E name: body_22 , SW name: Foot-5/Foot-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)


# Mate constraint: Coincident19 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: Tibia-2/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_25 , SW name: Foot-1/Foot-1 ,  SW ref.type:2 (2)

link_73 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.237206453695562,-0.00741167207816019,0.0161200000000003)
cB = chrono.ChVectorD(-0.251807852743245,-0.0936051375422572,0.0161200000000002)
dA = chrono.ChVectorD(1.5841022702561e-16,1.01827747580707e-15,-1)
dB = chrono.ChVectorD(-1.50823303745856e-16,9.56338459285404e-17,1)
link_73.Initialize(body_5,body_25,False,cA,cB,dB)
link_73.SetDistance(0)
link_73.SetName("Coincident19")
exported_items.append(link_73)

link_74 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.237206453695562,-0.00741167207816019,0.0161200000000003)
dA = chrono.ChVectorD(1.5841022702561e-16,1.01827747580707e-15,-1)
cB = chrono.ChVectorD(-0.251807852743245,-0.0936051375422572,0.0161200000000002)
dB = chrono.ChVectorD(-1.50823303745856e-16,9.56338459285404e-17,1)
link_74.SetFlipped(True)
link_74.Initialize(body_5,body_25,False,cA,cB,dA,dB)
link_74.SetName("Coincident19")
exported_items.append(link_74)


# Mate constraint: Coincident20 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: Tibia-2/Tibia side plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_25 , SW name: Foot-1/Foot-1 ,  SW ref.type:2 (2)

link_75 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.237206453695562,-0.117605137542257,-0.0183799999999998)
cB = chrono.ChVectorD(-0.251807852743245,-0.117605137542257,0.000120000000000082)
dA = chrono.ChVectorD(-7.21644966006352e-16,-1,-9.60068676723063e-16)
dB = chrono.ChVectorD(1.37067389633989e-31,1,-2.66534584175186e-16)
link_75.Initialize(body_5,body_25,False,cA,cB,dB)
link_75.SetDistance(0)
link_75.SetName("Coincident20")
exported_items.append(link_75)

link_76 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.237206453695562,-0.117605137542257,-0.0183799999999998)
dA = chrono.ChVectorD(-7.21644966006352e-16,-1,-9.60068676723063e-16)
cB = chrono.ChVectorD(-0.251807852743245,-0.117605137542257,0.000120000000000082)
dB = chrono.ChVectorD(1.37067389633989e-31,1,-2.66534584175186e-16)
link_76.SetFlipped(True)
link_76.Initialize(body_5,body_25,False,cA,cB,dA,dB)
link_76.SetName("Coincident20")
exported_items.append(link_76)


# Mate constraint: Coincident21 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: Tibia-2/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_25 , SW name: Foot-1/Foot-1 ,  SW ref.type:2 (2)

link_77 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.244807852743245,-0.0926051375422575,0.0186210000000002)
cB = chrono.ChVectorD(-0.244807852743245,-0.0936051375422572,0.000120000000000076)
dA = chrono.ChVectorD(-1,3.88578058618805e-16,-2.6093338589849e-16)
dB = chrono.ChVectorD(1,1.7127738786788e-33,1.10434262106419e-16)
link_77.Initialize(body_5,body_25,False,cA,cB,dB)
link_77.SetDistance(0)
link_77.SetName("Coincident21")
exported_items.append(link_77)

link_78 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.244807852743245,-0.0926051375422575,0.0186210000000002)
dA = chrono.ChVectorD(-1,3.88578058618805e-16,-2.6093338589849e-16)
cB = chrono.ChVectorD(-0.244807852743245,-0.0936051375422572,0.000120000000000076)
dB = chrono.ChVectorD(1,1.7127738786788e-33,1.10434262106419e-16)
link_78.SetFlipped(True)
link_78.Initialize(body_5,body_25,False,cA,cB,dA,dB)
link_78.SetName("Coincident21")
exported_items.append(link_78)


# Mate constraint: Angle21 [MatePlanarAngleDim] type:6 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: Body-1/Top body plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: Foot-6/Foot-1 ,  SW ref.type:2 (2)


# Mate constraint: Coincident22 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Tibia-1/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_21 , SW name: Foot-2/Foot-1 ,  SW ref.type:2 (2)

link_79 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.144056256338006,-0.00741167207816035,0.22785337896346)
cB = chrono.ChVectorD(-0.154381004619433,-0.0936051375422575,0.238178127244888)
dA = chrono.ChVectorD(-0.707106781186547,-1.11022302462516e-16,-0.707106781186548)
dB = chrono.ChVectorD(0.707106781186547,-4.20617961358948e-16,0.707106781186548)
link_79.Initialize(body_1,body_21,False,cA,cB,dB)
link_79.SetDistance(0)
link_79.SetName("Coincident22")
exported_items.append(link_79)

link_80 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.144056256338006,-0.00741167207816035,0.22785337896346)
dA = chrono.ChVectorD(-0.707106781186547,-1.11022302462516e-16,-0.707106781186548)
cB = chrono.ChVectorD(-0.154381004619433,-0.0936051375422575,0.238178127244888)
dB = chrono.ChVectorD(0.707106781186547,-4.20617961358948e-16,0.707106781186548)
link_80.SetFlipped(True)
link_80.Initialize(body_1,body_21,False,cA,cB,dA,dB)
link_80.SetName("Coincident22")
exported_items.append(link_80)


# Mate constraint: Coincident23 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Tibia-1/Tibia side plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_21 , SW name: Foot-2/Foot-1 ,  SW ref.type:2 (2)

link_81 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.168451440288941,-0.117605137542257,0.203458195012524)
cB = chrono.ChVectorD(-0.165694713118418,-0.117605137542257,0.226864418745903)
dA = chrono.ChVectorD(-5.55111512312578e-17,-1,5.55111512312578e-17)
dB = chrono.ChVectorD(1.04623861413478e-16,1,3.26668466338508e-16)
link_81.Initialize(body_1,body_21,False,cA,cB,dB)
link_81.SetDistance(0)
link_81.SetName("Coincident23")
exported_items.append(link_81)

link_82 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.168451440288941,-0.117605137542257,0.203458195012524)
dA = chrono.ChVectorD(-5.55111512312578e-17,-1,5.55111512312578e-17)
cB = chrono.ChVectorD(-0.165694713118418,-0.117605137542257,0.226864418745903)
dB = chrono.ChVectorD(1.04623861413478e-16,1,3.26668466338508e-16)
link_82.SetFlipped(True)
link_82.Initialize(body_1,body_21,False,cA,cB,dA,dB)
link_82.SetName("Coincident23")
exported_items.append(link_82)


# Mate constraint: Coincident24 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_1 , SW name: Tibia-1/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_21 , SW name: Foot-2/Foot-1 ,  SW ref.type:2 (2)

link_83 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.14766278309138,-0.0926051375422577,0.234996853836329)
cB = chrono.ChVectorD(-0.160744965650112,-0.0936051375422575,0.221914671277597)
dA = chrono.ChVectorD(-0.707106781186548,-1.66533453693773e-16,0.707106781186547)
dB = chrono.ChVectorD(0.707106781186548,1.57009245868376e-16,-0.707106781186547)
link_83.Initialize(body_1,body_21,False,cA,cB,dB)
link_83.SetDistance(0)
link_83.SetName("Coincident24")
exported_items.append(link_83)

link_84 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.14766278309138,-0.0926051375422577,0.234996853836329)
dA = chrono.ChVectorD(-0.707106781186548,-1.66533453693773e-16,0.707106781186547)
cB = chrono.ChVectorD(-0.160744965650112,-0.0936051375422575,0.221914671277597)
dB = chrono.ChVectorD(0.707106781186548,1.57009245868376e-16,-0.707106781186547)
link_84.SetFlipped(True)
link_84.Initialize(body_1,body_21,False,cA,cB,dA,dB)
link_84.SetName("Coincident24")
exported_items.append(link_84)


# Mate constraint: Coincident25 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: Tibia-3/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: Foot-3/Foot-1 ,  SW ref.type:2 (2)

link_85 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.166853378963403,-0.00741167207816017,-0.205056256338051)
cB = chrono.ChVectorD(-0.172228379776524,-0.0936051375422573,-0.210431257151175)
dA = chrono.ChVectorD(0.707106781186741,-4.36116585615091e-16,-0.707106781186354)
dB = chrono.ChVectorD(-0.707106781186741,-1.27709808046369e-15,0.707106781186354)
link_85.Initialize(body_11,body_20,False,cA,cB,dB)
link_85.SetDistance(0)
link_85.SetName("Coincident25")
exported_items.append(link_85)

link_86 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.166853378963403,-0.00741167207816017,-0.205056256338051)
dA = chrono.ChVectorD(0.707106781186741,-4.36116585615091e-16,-0.707106781186354)
cB = chrono.ChVectorD(-0.172228379776524,-0.0936051375422573,-0.210431257151175)
dB = chrono.ChVectorD(-0.707106781186741,-1.27709808046369e-15,0.707106781186354)
link_86.SetFlipped(True)
link_86.Initialize(body_11,body_20,False,cA,cB,dA,dB)
link_86.SetName("Coincident25")
exported_items.append(link_86)


# Mate constraint: Coincident26 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: Tibia-3/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: Foot-3/Foot-1 ,  SW ref.type:2 (2)

link_87 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.173996853836272,-0.0926051375422576,-0.208662783091428)
cB = chrono.ChVectorD(-0.160914671277536,-0.0936051375422573,-0.221744965650156)
dA = chrono.ChVectorD(-0.707106781186355,-8.88178419700125e-16,-0.707106781186741)
dB = chrono.ChVectorD(0.707106781186354,5.55111512312578e-17,0.707106781186741)
link_87.Initialize(body_11,body_20,False,cA,cB,dB)
link_87.SetDistance(0)
link_87.SetName("Coincident26")
exported_items.append(link_87)

link_88 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.173996853836272,-0.0926051375422576,-0.208662783091428)
dA = chrono.ChVectorD(-0.707106781186355,-8.88178419700125e-16,-0.707106781186741)
cB = chrono.ChVectorD(-0.160914671277536,-0.0936051375422573,-0.221744965650156)
dB = chrono.ChVectorD(0.707106781186354,5.55111512312578e-17,0.707106781186741)
link_88.SetFlipped(True)
link_88.Initialize(body_11,body_20,False,cA,cB,dA,dB)
link_88.SetName("Coincident26")
exported_items.append(link_88)


# Mate constraint: Coincident27 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: Tibia-3/Tibia side plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_20 , SW name: Foot-3/Foot-1 ,  SW ref.type:2 (2)

link_89 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(-0.142458195012461,-0.117605137542257,-0.22945144028898)
cB = chrono.ChVectorD(-0.160914671277536,-0.117605137542257,-0.221744965650156)
dA = chrono.ChVectorD(2.22044604925031e-16,-1,9.43689570931383e-16)
dB = chrono.ChVectorD(-7.52587209573939e-16,1,6.97076058342209e-16)
link_89.Initialize(body_11,body_20,False,cA,cB,dB)
link_89.SetDistance(0)
link_89.SetName("Coincident27")
exported_items.append(link_89)

link_90 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(-0.142458195012461,-0.117605137542257,-0.22945144028898)
dA = chrono.ChVectorD(2.22044604925031e-16,-1,9.43689570931383e-16)
cB = chrono.ChVectorD(-0.160914671277536,-0.117605137542257,-0.221744965650156)
dB = chrono.ChVectorD(-7.52587209573939e-16,1,6.97076058342209e-16)
link_90.SetFlipped(True)
link_90.Initialize(body_11,body_20,False,cA,cB,dA,dB)
link_90.SetName("Coincident27")
exported_items.append(link_90)


# Mate constraint: Coincident28 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: Tibia-6/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: Foot-4/Foot-1 ,  SW ref.type:2 (2)

link_91 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.14766278309138,-0.0926051375422576,-0.234996853836329)
cB = chrono.ChVectorD(0.160744965650112,-0.0936051375422574,-0.221914671277597)
dA = chrono.ChVectorD(0.707106781186548,-2.38697950294409e-15,-0.707106781186547)
dB = chrono.ChVectorD(-0.707106781186548,-3.92523114670933e-17,0.707106781186547)
link_91.Initialize(body_13,body_23,False,cA,cB,dB)
link_91.SetDistance(0)
link_91.SetName("Coincident28")
exported_items.append(link_91)

link_92 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.14766278309138,-0.0926051375422576,-0.234996853836329)
dA = chrono.ChVectorD(0.707106781186548,-2.38697950294409e-15,-0.707106781186547)
cB = chrono.ChVectorD(0.160744965650112,-0.0936051375422574,-0.221914671277597)
dB = chrono.ChVectorD(-0.707106781186548,-3.92523114670933e-17,0.707106781186547)
link_92.SetFlipped(True)
link_92.Initialize(body_13,body_23,False,cA,cB,dA,dB)
link_92.SetName("Coincident28")
exported_items.append(link_92)


# Mate constraint: Coincident29 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: Tibia-6/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: Foot-4/Foot-1 ,  SW ref.type:2 (2)

link_93 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.144056256338005,-0.117605137542257,-0.22785337896346)
cB = chrono.ChVectorD(0.160744965650112,-0.117605137542257,-0.221914671277597)
dA = chrono.ChVectorD(-2.27595720048157e-15,-1,7.7715611723761e-16)
dB = chrono.ChVectorD(6.78544481599045e-16,1,7.34055632830302e-16)
link_93.Initialize(body_13,body_23,False,cA,cB,dB)
link_93.SetDistance(0)
link_93.SetName("Coincident29")
exported_items.append(link_93)

link_94 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.144056256338005,-0.117605137542257,-0.22785337896346)
dA = chrono.ChVectorD(-2.27595720048157e-15,-1,7.7715611723761e-16)
cB = chrono.ChVectorD(0.160744965650112,-0.117605137542257,-0.221914671277597)
dB = chrono.ChVectorD(6.78544481599045e-16,1,7.34055632830302e-16)
link_94.SetFlipped(True)
link_94.Initialize(body_13,body_23,False,cA,cB,dA,dB)
link_94.SetName("Coincident29")
exported_items.append(link_94)


# Mate constraint: Coincident30 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: Tibia-6/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_23 , SW name: Foot-4/Foot-1 ,  SW ref.type:2 (2)

link_95 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.144056256338006,-0.00741167207816025,-0.22785337896346)
cB = chrono.ChVectorD(0.149431257151127,-0.0936051375422574,-0.233228379776582)
dA = chrono.ChVectorD(0.707106781186547,-1.02695629777827e-15,0.707106781186548)
dB = chrono.ChVectorD(-0.707106781186547,9.98859120017884e-16,-0.707106781186548)
link_95.Initialize(body_13,body_23,False,cA,cB,dB)
link_95.SetDistance(0)
link_95.SetName("Coincident30")
exported_items.append(link_95)

link_96 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.144056256338006,-0.00741167207816025,-0.22785337896346)
dA = chrono.ChVectorD(0.707106781186547,-1.02695629777827e-15,0.707106781186548)
cB = chrono.ChVectorD(0.149431257151127,-0.0936051375422574,-0.233228379776582)
dB = chrono.ChVectorD(-0.707106781186547,9.98859120017884e-16,-0.707106781186548)
link_96.SetFlipped(True)
link_96.Initialize(body_13,body_23,False,cA,cB,dA,dB)
link_96.SetName("Coincident30")
exported_items.append(link_96)


# Mate constraint: Coincident31 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_12 , SW name: Tibia-5/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: Foot-5/Foot-1 ,  SW ref.type:2 (2)

link_97 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.237206453695562,-0.00741167207816043,-0.01612)
cB = chrono.ChVectorD(0.244807852743245,-0.0936051375422575,-0.0161200000000001)
dA = chrono.ChVectorD(-5.01446141708683e-16,-7.66979284412761e-16,1)
dB = chrono.ChVectorD(6.60845056602136e-16,4.13160837432814e-16,-1)
link_97.Initialize(body_12,body_22,False,cA,cB,dB)
link_97.SetDistance(0)
link_97.SetName("Coincident31")
exported_items.append(link_97)

link_98 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.237206453695562,-0.00741167207816043,-0.01612)
dA = chrono.ChVectorD(-5.01446141708683e-16,-7.66979284412761e-16,1)
cB = chrono.ChVectorD(0.244807852743245,-0.0936051375422575,-0.0161200000000001)
dB = chrono.ChVectorD(6.60845056602136e-16,4.13160837432814e-16,-1)
link_98.SetFlipped(True)
link_98.Initialize(body_12,body_22,False,cA,cB,dA,dB)
link_98.SetName("Coincident31")
exported_items.append(link_98)


# Mate constraint: Coincident32 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_12 , SW name: Tibia-5/Tibia side plate-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: Foot-5/Foot-1 ,  SW ref.type:2 (2)

link_99 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.237206453695562,-0.117605137542257,0.0183799999999999)
cB = chrono.ChVectorD(0.244807852743245,-0.117605137542257,-0.000119999999999968)
dA = chrono.ChVectorD(5.55111512312578e-17,-1,-6.24896187932046e-16)
dB = chrono.ChVectorD(-5.55111512312571e-17,1,4.13160837432814e-16)
link_99.Initialize(body_12,body_22,False,cA,cB,dB)
link_99.SetDistance(0)
link_99.SetName("Coincident32")
exported_items.append(link_99)

link_100 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.237206453695562,-0.117605137542257,0.0183799999999999)
dA = chrono.ChVectorD(5.55111512312578e-17,-1,-6.24896187932046e-16)
cB = chrono.ChVectorD(0.244807852743245,-0.117605137542257,-0.000119999999999968)
dB = chrono.ChVectorD(-5.55111512312571e-17,1,4.13160837432814e-16)
link_100.SetFlipped(True)
link_100.Initialize(body_12,body_22,False,cA,cB,dA,dB)
link_100.SetName("Coincident32")
exported_items.append(link_100)


# Mate constraint: Coincident33 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_12 , SW name: Tibia-5/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_22 , SW name: Foot-5/Foot-1 ,  SW ref.type:2 (2)

link_101 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.244807852743245,-0.0926051375422578,-0.0186210000000001)
cB = chrono.ChVectorD(0.244807852743245,-0.0936051375422575,-0.000119999999999958)
dA = chrono.ChVectorD(1,-3.33066907387547e-16,5.01446141708683e-16)
dB = chrono.ChVectorD(-1,-5.55111512312568e-17,-6.60845056602136e-16)
link_101.Initialize(body_12,body_22,False,cA,cB,dB)
link_101.SetDistance(0)
link_101.SetName("Coincident33")
exported_items.append(link_101)

link_102 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.244807852743245,-0.0926051375422578,-0.0186210000000001)
dA = chrono.ChVectorD(1,-3.33066907387547e-16,5.01446141708683e-16)
cB = chrono.ChVectorD(0.244807852743245,-0.0936051375422575,-0.000119999999999958)
dB = chrono.ChVectorD(-1,-5.55111512312568e-17,-6.60845056602136e-16)
link_102.SetFlipped(True)
link_102.Initialize(body_12,body_22,False,cA,cB,dA,dB)
link_102.SetName("Coincident33")
exported_items.append(link_102)


# Mate constraint: Coincident34 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Tibia-4/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: Foot-6/Foot-1 ,  SW ref.type:2 (2)

link_103 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.16685337896346,-0.00741167207816037,0.205056256338005)
cB = chrono.ChVectorD(0.172228379776582,-0.0936051375422575,0.210431257151127)
dA = chrono.ChVectorD(-0.707106781186545,7.97262423497152e-18,0.70710678118655)
dB = chrono.ChVectorD(0.707106781186546,7.25167838462683e-17,-0.70710678118655)
link_103.Initialize(body_9,body_24,False,cA,cB,dB)
link_103.SetDistance(0)
link_103.SetName("Coincident34")
exported_items.append(link_103)

link_104 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.16685337896346,-0.00741167207816037,0.205056256338005)
dA = chrono.ChVectorD(-0.707106781186545,7.97262423497152e-18,0.70710678118655)
cB = chrono.ChVectorD(0.172228379776582,-0.0936051375422575,0.210431257151127)
dB = chrono.ChVectorD(0.707106781186546,7.25167838462683e-17,-0.70710678118655)
link_104.SetFlipped(True)
link_104.Initialize(body_9,body_24,False,cA,cB,dA,dB)
link_104.SetName("Coincident34")
exported_items.append(link_104)


# Mate constraint: Coincident36 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Tibia-4/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: Foot-6/Foot-1 ,  SW ref.type:2 (2)

link_105 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.173996853836329,-0.0926051375422578,0.20866278309138)
cB = chrono.ChVectorD(0.160914671277597,-0.0936051375422575,0.221744965650112)
dA = chrono.ChVectorD(0.70710678118655,-1.72084568816899e-15,0.707106781186545)
dB = chrono.ChVectorD(-0.707106781186549,5.55111512312578e-17,-0.707106781186546)
link_105.Initialize(body_9,body_24,False,cA,cB,dB)
link_105.SetDistance(0)
link_105.SetName("Coincident36")
exported_items.append(link_105)

link_106 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.173996853836329,-0.0926051375422578,0.20866278309138)
dA = chrono.ChVectorD(0.70710678118655,-1.72084568816899e-15,0.707106781186545)
cB = chrono.ChVectorD(0.160914671277597,-0.0936051375422575,0.221744965650112)
dB = chrono.ChVectorD(-0.707106781186549,5.55111512312578e-17,-0.707106781186546)
link_106.SetFlipped(True)
link_106.Initialize(body_9,body_24,False,cA,cB,dA,dB)
link_106.SetName("Coincident36")
exported_items.append(link_106)


# Mate constraint: Coincident37 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: Tibia-4/Tibia side plate-3 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_24 , SW name: Foot-6/Foot-1 ,  SW ref.type:2 (2)

link_107 = chrono.ChLinkMateXdistance()
cA = chrono.ChVectorD(0.16685337896346,-0.117605137542257,0.205056256338005)
cB = chrono.ChVectorD(0.160914671277597,-0.117605137542257,0.221744965650112)
dA = chrono.ChVectorD(-7.21644966006352e-16,-1,-9.99200722162641e-16)
dB = chrono.ChVectorD(3.09422435673483e-17,1,2.45689076639048e-17)
link_107.Initialize(body_9,body_24,False,cA,cB,dB)
link_107.SetDistance(0)
link_107.SetName("Coincident37")
exported_items.append(link_107)

link_108 = chrono.ChLinkMateParallel()
cA = chrono.ChVectorD(0.16685337896346,-0.117605137542257,0.205056256338005)
dA = chrono.ChVectorD(-7.21644966006352e-16,-1,-9.99200722162641e-16)
cB = chrono.ChVectorD(0.160914671277597,-0.117605137542257,0.221744965650112)
dB = chrono.ChVectorD(3.09422435673483e-17,1,2.45689076639048e-17)
link_108.SetFlipped(True)
link_108.Initialize(body_9,body_24,False,cA,cB,dA,dB)
link_108.SetName("Coincident37")
exported_items.append(link_108)

