/*---------------------------------------------------------------------------
 *
 * Copyright 2016 by Kitware, Inc. All Rights Reserved. Please refer to
 *
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 *
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 *
 * Author : Chengjiang Long <chengjiang.long@kitware.com>
 *
 * Description : Visualize the world created by Gazebo in sdf format with VTK.
 *
 *
 * Created : <2016-06-30>
 *
 *-------------------------------------------------------------------------*/

#include <OgreRoot.h>
#include <OgreResourceGroupManager.h>
#include <OgreMaterialManager.h>
#include <OgreScriptCompiler.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgrePass.h>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "sdf/sdf.hh"

#include <vtkSmartPointer.h>
#include <vtkOBJReader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkAxesActor.h>

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkBoundingBox.h>

#include "vtksys/SystemTools.hxx"
#include <vtkOBJImporter.h>
#include <vtkJPEGReader.h>
#include <vtkPNGReader.h>
#include <vtkTexture.h>
#include <vtkCamera.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkBMPWriter.h>
#include <vtkImageShiftScale.h>

#include <vtkMath.h>
#include <vtkArrowSource.h>
#include <vtkMatrix4x4.h>
#include <vtkPlaneSource.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkProperty.h>
#include <vtkTextureMapToCylinder.h>
#include <vtkTextureMapToSphere.h>
#include <vtkTransformTextureCoords.h>
#include <vtkLight.h>
#include <vtkLightActor.h>

#include <unordered_map>
#include <unistd.h>
#define GetCurrentDir getcwd

using namespace std;

//-----------------------------------------------------------------------------------//
// Check whether the file exists.
//-----------------------------------------------------------------------------------//
bool CheckFileExist(const char *fileName)
{
	std::ifstream infile(fileName);
	return infile.good();
}

string GetCurrentPath()
{
	char the_path[256];
	getcwd(the_path, 255);
	string curPath(the_path);
	cout << "Current path: " << curPath << endl;

	return curPath;
}

//-----------------------------------------------------------------------------------//
// Find a sub-string in the source string, and then replace with another sub-string.
//-----------------------------------------------------------------------------------//
void FindAndReplace(string& source, string const& find, string const& replace)
{
	for(string::size_type i = 0; (i = source.find(find, i)) != string::npos;)
	{
		source.replace(i, find.length(), replace);
		i += replace.length();
	}
}

//-----------------------------------------------------------------------------------//
// Covert the pose element in the SDF format into vtkTransform.
//-----------------------------------------------------------------------------------//
vtkSmartPointer<vtkTransform> ConvertPoseToVTKTransform(sdf::ElementPtr elem, bool bPostMult)
{
	ignition::math::Pose3d pose = elem->GetElement("pose")->Get<ignition::math::Pose3d>();
	ignition::math::Vector3d posVec = pose.Pos();
	ignition::math::Quaterniond rotQuat = pose.Rot();
	ignition::math::Vector3d rotVec = rotQuat.Euler();
	cout << "Pos : " << posVec[0] << " , " << posVec[1] << " , " << posVec[2] << endl;
	cout << "Rot : " << rotVec[0] << " , " << rotVec[1] << " , " << rotVec[2] << endl;
	
	// Tranform the objects
	double normal = rotQuat.X()*rotQuat.X() + rotQuat.Y()*rotQuat.Y() + rotQuat.Z()*rotQuat.Z();
	if(normal > 0)
	{
		rotQuat.X() = rotQuat.X()/sqrt(normal);
		rotQuat.Y() = rotQuat.Y()/sqrt(normal);
		rotQuat.Z() = rotQuat.Z()/sqrt(normal);
	}

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	if(normal > 0)
	{
		transform->RotateWXYZ(rotQuat.W(), rotQuat.X(), rotQuat.Y(), rotQuat.Z());
	}
	transform->Translate(posVec[0], posVec[1], posVec[2]);
    
	if(bPostMult == true)
	{
		transform->PostMultiply();
	}
	
	transform->Update();

	return transform;
}

//-----------------------------------------------------------------------------------//
// A world can have only one state element, as sated in the web page (URL:
// http://sdformat.org/spec?ver=1.6&elem=state).
// Save the vtkTransforms from the poses stored in the state element into a hash map. 
//-----------------------------------------------------------------------------------//
unordered_map<string, vtkSmartPointer<vtkTransform> > GetStateModelTransformHash(sdf::ElementPtr state)
{
	unordered_map<string, vtkSmartPointer<vtkTransform> > mHash_Transform;
	
	for(sdf::ElementPtr model = state->GetElement("model"); model; model = model->GetNextElement("model"))
	{
		string model_name = model->Get<std::string>("name");
		cout << "model_name: " << model_name << endl;
	
		// Get the pose for the model.
		vtkSmartPointer<vtkTransform> mTransform = ConvertPoseToVTKTransform(model, "false");
		mHash_Transform[model_name] = mTransform;	
	}

	return mHash_Transform;
}

//-----------------------------------------------------------------------------------//
// Add the SDF material to vtkActor.
//-----------------------------------------------------------------------------------//
void AddSdfMaterailToVTKActor(vtkSmartPointer<vtkActor> actor, sdf::ElementPtr mtlPtr)
{
	vtkSmartPointer<vtkProperty> objMtl = vtkSmartPointer<vtkProperty>::New();
	//objMtl->SetAmbient(0.3333);
	//objMtl->SetDiffuse(0.3333);
	//objMtl->SetSpecular(0.3333);
	
	if(mtlPtr->HasElement("ambient"))
	{
		ignition::math::Vector4d acolor = mtlPtr->GetElement("ambient")->Get<ignition::math::Vector4d>();
		cout << "Ambient color : " << acolor[0] << " , " << acolor[1] << " , " << acolor[2] << endl;
		objMtl->SetAmbientColor(acolor[0], acolor[1], acolor[2]);
	}
	
	if(mtlPtr->HasElement("diffuse"))
	{
		ignition::math::Vector4d dcolor = mtlPtr->GetElement("diffuse")->Get<ignition::math::Vector4d>();
		cout << "Diffuse color : " << dcolor[0] << " , " << dcolor[1] << " , " << dcolor[2] << endl;
		objMtl->SetDiffuseColor(dcolor[0], dcolor[1], dcolor[2]);
	}
	
	if(mtlPtr->HasElement("specular"))
	{
		ignition::math::Vector4d scolor = mtlPtr->GetElement("specular")->Get<ignition::math::Vector4d>();
		cout << "Specular color : " << scolor[0] << " , " << scolor[1] << " , " << scolor[2] << endl;
		objMtl->SetSpecularColor(scolor[0],scolor[1], scolor[2]);
	}

	actor->SetProperty(objMtl);

}

//-----------------------------------------------------------------------------------//
// Add the OGRE material into vtkActor.
//-----------------------------------------------------------------------------------//
void AddOgreMaterailToVTKActor(vtkSmartPointer<vtkActor> actor, string mtlname, string parentDir)
{
	Ogre::MaterialPtr materialPtr = Ogre::MaterialManager::getSingleton().getByName(mtlname);

	std::cout << " MaterialPtr size: " << materialPtr->getSize() << std::endl;
	
	Ogre::ColourValue acolor = materialPtr->getTechnique(0)->getPass(0)->getAmbient();
	std::cout << " Ambient Color: " << acolor[0] << ", " << acolor[1] << ", " << acolor[2] << ", " << acolor[3] << std::endl;
	
	Ogre::ColourValue dcolor = materialPtr->getTechnique(0)->getPass(0)->getDiffuse(); 
	std::cout << " Diffuse Color: " << dcolor[0] << ", " << dcolor[1] << ", " << dcolor[2] << ", " << dcolor[3] << std::endl;

	Ogre::ColourValue scolor = materialPtr->getTechnique(0)->getPass(0)->getSpecular();
	std::cout << " Specular Color: " << scolor[0] << ", " << scolor[1] << ", " << scolor[2] << ", " << scolor[3] << std::endl;

	vtkSmartPointer<vtkProperty> objMtl = vtkSmartPointer<vtkProperty>::New();
	//objMtl->SetAmbient(0.3333);
	//objMtl->SetDiffuse(0.3333);
	//objMtl->SetSpecular(0.3333);
	objMtl->SetAmbientColor(acolor[0], acolor[1], acolor[2]);
	objMtl->SetDiffuseColor(dcolor[0], dcolor[1], dcolor[2]);
	objMtl->SetSpecularColor(scolor[0],scolor[1], scolor[2]);


	// Texture
	int textnum = materialPtr->getTechnique(0)->getPass(0)->getNumTextureUnitStates();
	if(textnum == 1)
	{
		Ogre::TextureUnitState* ptus = materialPtr->getTechnique(0)->getPass(0)->getTextureUnitState(0); 
		std::cout << " Texture Name: " << ptus->getTextureName() << std::endl;
		string textimgname = ptus->getTextureName();
		if(!CheckFileExist(textimgname.c_str()))
		{
			textimgname = parentDir + ptus->getTextureName();
			//cout << "No such a textrue image. Please take a check. " << endl;
			//exit(0);
		}

		string imgtype = textimgname.substr(textimgname.length()-4);
		cout << "imgtype: " << imgtype << endl;

		vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
		if(imgtype == ".jpg" || imgtype == ".JPG")
		{
			cout << "using vtkPNGReader. " << endl;
			vtkSmartPointer<vtkJPEGReader> jPEGReader = vtkSmartPointer<vtkJPEGReader>::New();
			jPEGReader->SetFileName(textimgname.c_str());
			jPEGReader->Update();
			
			texture->SetInputConnection(jPEGReader->GetOutputPort());
		}

		if(imgtype == ".png" || imgtype == ".PNG")
		{
			cout << "using vtkPNGReader. " << endl;
			vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
			pngReader->SetFileName(textimgname.c_str());
			pngReader->Update();
			
			texture->SetInputConnection(pngReader->GetOutputPort());
		}

		actor->SetTexture(texture);
	}
		
	actor->SetProperty(objMtl);
							
}


//-----------------------------------------------------------------------------------//
// Output the boudning box vtkActor given the parameter "double *bbox".
//-----------------------------------------------------------------------------------//
vtkSmartPointer<vtkActor> BoundingBoxVTKActor(double *bbox)
{
	vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
	for(int xi=0; xi<2; xi++)
	{
		for(int yi=2; yi<4; yi++)
		{
			for(int zi=4; zi<6; zi++)
			{
				double pt[3];
				pt[0] = bbox[xi];
				pt[1] = bbox[yi];
				pt[2] = bbox[zi];

				pts->InsertNextPoint(pt);
			}
		}
	}

	linesPolyData->SetPoints(pts);

    vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
	line0->GetPointIds()->SetId(0, 0);
	line0->GetPointIds()->SetId(1, 1);

    vtkSmartPointer<vtkLine> line1 = vtkSmartPointer<vtkLine>::New();
	line1->GetPointIds()->SetId(0, 0);
	line1->GetPointIds()->SetId(1, 2);

    vtkSmartPointer<vtkLine> line2 = vtkSmartPointer<vtkLine>::New();
	line2->GetPointIds()->SetId(0, 0);
	line2->GetPointIds()->SetId(1, 4);

    vtkSmartPointer<vtkLine> line3 = vtkSmartPointer<vtkLine>::New();
	line3->GetPointIds()->SetId(0, 1);
	line3->GetPointIds()->SetId(1, 3);

    vtkSmartPointer<vtkLine> line4 = vtkSmartPointer<vtkLine>::New();
	line4->GetPointIds()->SetId(0, 1);
	line4->GetPointIds()->SetId(1, 5);

    vtkSmartPointer<vtkLine> line5 = vtkSmartPointer<vtkLine>::New();
	line5->GetPointIds()->SetId(0, 2);
	line5->GetPointIds()->SetId(1, 3);

    vtkSmartPointer<vtkLine> line6 = vtkSmartPointer<vtkLine>::New();
	line6->GetPointIds()->SetId(0, 2);
	line6->GetPointIds()->SetId(1, 6);

    vtkSmartPointer<vtkLine> line7 = vtkSmartPointer<vtkLine>::New();
	line7->GetPointIds()->SetId(0, 3);
	line7->GetPointIds()->SetId(1, 7);

    vtkSmartPointer<vtkLine> line8 = vtkSmartPointer<vtkLine>::New();
	line8->GetPointIds()->SetId(0, 4);
	line8->GetPointIds()->SetId(1, 5);

    vtkSmartPointer<vtkLine> line9 = vtkSmartPointer<vtkLine>::New();
	line9->GetPointIds()->SetId(0, 4);
	line9->GetPointIds()->SetId(1, 6);

    vtkSmartPointer<vtkLine> line10 = vtkSmartPointer<vtkLine>::New();
	line10->GetPointIds()->SetId(0, 5);
	line10->GetPointIds()->SetId(1, 7);

    vtkSmartPointer<vtkLine> line11 = vtkSmartPointer<vtkLine>::New();
	line11->GetPointIds()->SetId(0, 6);
	line11->GetPointIds()->SetId(1, 7);

	vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
	lines->InsertNextCell(line0);
	lines->InsertNextCell(line1);
	lines->InsertNextCell(line2);
	lines->InsertNextCell(line3);
	lines->InsertNextCell(line4);
	lines->InsertNextCell(line5);
	lines->InsertNextCell(line6);
	lines->InsertNextCell(line7);
	lines->InsertNextCell(line8);
	lines->InsertNextCell(line9);
	lines->InsertNextCell(line10);
	lines->InsertNextCell(line11);

	linesPolyData->SetLines(lines);

    vtkSmartPointer<vtkPolyDataMapper> linemapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	linemapper->SetInputData(linesPolyData);

	vtkSmartPointer<vtkActor> bboxActor = vtkSmartPointer<vtkActor>::New();
	bboxActor->SetMapper(linemapper);

	return bboxActor;
}

//-----------------------------------------------------------------------------------//
// Main function.
//-----------------------------------------------------------------------------------//
int main(int argc, char *argv[])
{
	bool bTakeRGBD = false;
	if(argc != 3)
	{
		std::cerr << "Usage: " << argv[0]  << " sdfile bTakeRGBD(0:no, 1:yes)." << std::endl;
		return EXIT_FAILURE;
    }
	
	const std::string sdfile = argv[1];
    //const std::string sdfile = "../worlds/beerstable.world";
    //const std::string sdfile = "../worlds/cafebeers.world";
    //const std::string sdfile = "../worlds/realscene.world";

	if(atoi(argv[2]) == 1)
	{
		bTakeRGBD = true;
	}

	string curPath = GetCurrentPath();
	
	//----------------------------------------------------------//
	// Read the Ogre material.
	//----------------------------------------------------------//
	Ogre::Root *root = new Ogre::Root();	
	Ogre::ScriptCompiler* scriptcompiler = new Ogre::ScriptCompiler();

	std::string mtlfile = "../worlds/gazebo.material";
	
	std::ifstream sfile(mtlfile.c_str());
    Ogre::DataStreamPtr dataStream(OGRE_NEW Ogre::FileStreamDataStream(&sfile,false));  // setting "true" will cause free(): invalid pointer, Aborted (core dumped).

	scriptcompiler->compile(dataStream->getAsString(),"",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

	
	string parentDir = curPath + "/..";

	std::ostringstream stream;
	string line;
	ifstream myfile(sdfile);
	if (myfile.is_open())
	{
		while (! myfile.eof() )
		{
			getline (myfile,line);
			
			//cout << line << endl;
			stream << line;
																 
		}
		myfile.close();
	}
	else 
	{
		cout << "Unable to open file"; 
	}


	sdf::SDFPtr robot(new sdf::SDF());
	sdf::init(robot);
    assert(sdf::readString(stream.str(), robot));

	sdf::ElementPtr world;
	
	if(robot->Root()->HasElement("world"))
	{
		world = robot->Root()->GetElement("world");
	}
	else
	{
		world = robot->Root();
	}

	//world->PrintDescription("world");
	//world->PrintValues("model");
	
	vtkSmartPointer<vtkRenderer> renderer =  vtkSmartPointer<vtkRenderer>::New();

	vtkBoundingBox boundingbox;

	bool bHasBeerMaterial = false;
	
	int light_count = 0;
	for(sdf::ElementPtr light = world->GetElement("light"); light; light = light->GetNextElement("light"))
	{
		light_count++;
		cout << "light_count = " << light_count << endl;
		//model->PrintValues("model");

		// Get the pose for the model.
		vtkSmartPointer<vtkTransform> lTransform = ConvertPoseToVTKTransform(light, false); 

	    int bShadow = light->GetElement("cast_shadows")->Get<int>();	
		ignition::math::Vector4d diffuse = light->GetElement("diffuse")->Get<ignition::math::Vector4d>();
		ignition::math::Vector4d specular = light->GetElement("specular")->Get<ignition::math::Vector4d>();
		ignition::math::Vector3d direction = light->GetElement("direction")->Get<ignition::math::Vector3d>();
		double range = light->GetElement("attenuation")->GetElement("range")->Get<double>();
		double dconstant = light->GetElement("attenuation")->GetElement("constant")->Get<double>();
		double dlinear = light->GetElement("attenuation")->GetElement("linear")->Get<double>();
		double dquadratic = light->GetElement("attenuation")->GetElement("quadratic")->Get<double>();

		cout << "cast_shadows:  " << bShadow << endl;
		cout << "diffuse:  " << diffuse[0] << " , " << diffuse[1] << " , " << diffuse[2] << " , " << diffuse[3] << endl; 
		cout << "specular:  " << specular[0] << " , " << specular[1] << " , " << specular[2] << " , " << specular[3] << endl; 
		cout << "direction:  " << direction[0] << " , " << direction[1] << " , " << direction[2] << endl; 
        cout << "range:  " << range << endl;
        cout << "constant:  " << dconstant << endl;
        cout << "linear:  " << dlinear << endl;
        cout << "quadratic:  " << dquadratic << endl;

		double *pos = lTransform->GetPosition(); 

		vtkSmartPointer<vtkLight> lt = vtkSmartPointer<vtkLight>::New();
		lt->SetPosition(pos[0], pos[1], pos[2]);
		lt->SetDiffuseColor(diffuse[0], diffuse[1], diffuse[2]);
		lt->SetSpecularColor(specular[0], specular[1], specular[2]);
		lt->SetAttenuationValues(dconstant, dlinear, dquadratic);
		lt->SetFocalPoint(pos[0]+direction[0], pos[1]+direction[1], pos[2]+direction[2]);
		lt->SwitchOn();
	    
		renderer->AddLight(lt);

	}

    sdf::ElementPtr state = world->GetElement("state");
	state->PrintValues("state");

	// Get the model transforms hash table at the state.
	unordered_map<string, vtkSmartPointer<vtkTransform> > mHash_Transform = GetStateModelTransformHash(state);
    
	int model_count = 0;
	for(sdf::ElementPtr model = world->GetElement("model"); model; model = model->GetNextElement("model"))
	{
		model_count++;
		cout << "model_count = " << model_count << endl;
		string model_name = model->Get<std::string>("name");
        vtkSmartPointer<vtkTransform> gTransform = mHash_Transform[model_name];

		// Extract links for each model.
	    int count = 0;	
		for(sdf::ElementPtr obj = model->GetElement("link"); obj; obj = obj->GetNextElement("link"))
		{
			count++;
			obj->PrintValues("link");
			cout << "count = " << count << endl;
			

			cout <<"Link name: " << obj->Get<std::string>("name") << endl;
			
			//Extract the transform for the link.
			vtkSmartPointer<vtkTransform> transform = ConvertPoseToVTKTransform(obj, true);


			transform->Concatenate(gTransform->GetMatrix());
			transform->Update();

			// Load the objects.
			for(sdf::ElementPtr vis = obj->GetElement("visual"); vis; vis = vis->GetNextElement("visual"))
			{
                cout << "Visual name:  " << vis->Get<std::string>("name") << endl;
				sdf::ElementPtr geo = vis->GetElement("geometry");

				bool bPlane = geo->HasElement("plane");
				bool bBox = geo->HasElement("box");
				bool bCylinder = geo->HasElement("cylinder");
				bool bMesh = geo->HasElement("mesh");

				if(bPlane)
				{
					cout << "This link has a plane. " << endl;
				}

				if(bBox)
				{
					cout << "This link has a box. " << endl;
				}

				if(bCylinder)
				{
					cout << "This link has a cylinder. " << endl;
				}

				if(bMesh)
				{
					cout << "This link has a mesh. " << endl;
				}

				ignition::math::Pose3d pose = vis->GetElement("pose")->Get<ignition::math::Pose3d>();
				ignition::math::Vector3d posVec = pose.Pos();
				ignition::math::Quaterniond rotQuat = pose.Rot();
				ignition::math::Vector3d rotVec = rotQuat.Euler();
				cout << "Visual Pos : " << posVec[0] << " , " << posVec[1] << " , " << posVec[2] << endl;
				cout << "Visual Rot : " << rotVec[0] << " , " << rotVec[1] << " , " << rotVec[2] << endl;
				
				
				// Tranform the objects
				
				double normal = rotQuat.X()*rotQuat.X() + rotQuat.Y()*rotQuat.Y() + rotQuat.Z()*rotQuat.Z();
				if(normal > 0)
				{
					rotQuat.X() = rotQuat.X()/sqrt(normal);
					rotQuat.Y() = rotQuat.Y()/sqrt(normal);
					rotQuat.Z() = rotQuat.Z()/sqrt(normal);
				}
#include <vtkPNGReader.h>
				vtkSmartPointer<vtkTransform> visTransform = vtkSmartPointer<vtkTransform>::New();
				if(normal > 0)
				{
					visTransform->RotateWXYZ(rotQuat.W(), rotQuat.X(), rotQuat.Y(), rotQuat.Z());
				}
				visTransform->Translate(posVec[0], posVec[1], posVec[2]);
				//transform->PreMultiply();
				visTransform->PostMultiply();
				visTransform->Update();


				visTransform->Concatenate(transform->GetMatrix());
				visTransform->Update();


				bool bHasMaterial = vis->HasElement("material");
				//sdf::ElementPtr mtl = obj->GetElement("visual")->GetElement("material");
					
				if(geo->HasElement("plane") == true)
				{
					sdf::ElementPtr plane = geo->GetElement("plane");
					ignition::math::Vector3d normalVec = plane->GetElement("normal")->Get<ignition::math::Vector3d>();
					ignition::math::Vector2d sizeVec = plane->GetElement("size")->Get<ignition::math::Vector2d>();
					cout << "Plane normal: " << normalVec[0] << " , " << normalVec[1] << " , " << normalVec[2] << endl;
					cout << "Plane size normal: " << sizeVec[0] << " , " << sizeVec[1] << endl;
                    
                    if(sizeVec[0] > 1 && sizeVec[1] > 1)
					{
						vtkSmartPointer<vtkPlaneSource> planeSource =  vtkSmartPointer<vtkPlaneSource>::New();
						planeSource->SetCenter(-sizeVec[0]*0.5, -sizeVec[1]*0.5, -0.073);
						planeSource->SetNormal(normalVec[0], normalVec[1], normalVec[2]);
						planeSource->SetPoint1(sizeVec[0]*0.5, -sizeVec[1]*0.5, -0.073);
						planeSource->SetPoint2(-sizeVec[0]*0.5, sizeVec[1]*0.5, -0.073);
						planeSource->Update();

						// Create a mapper and actor
						vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
						mapper->SetInputData(planeSource->GetOutput());          
						vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
						actor->SetMapper(mapper);
						actor->SetUserTransform(visTransform);

						if(bHasMaterial)
						{
							string mtlname = obj->GetElement("visual")->GetElement("material")->GetElement("script")->GetElement("name")->Get<string>();
							cout << "mtlname: " << mtlname << endl;
							AddOgreMaterailToVTKActor(actor, mtlname, parentDir);
						}

						//renderer->AddActor(actor);
						//double *bounds = actor->GetBounds();	
						//boundingbox.AddBounds(bounds);
					}
				}

				if(geo->HasElement("box") == true)
				{
					sdf::ElementPtr box = geo->GetElement("box");
					ignition::math::Vector3d sizeVec = box->GetElement("size")->Get<ignition::math::Vector3d>();
					cout << "Box size normal: " << sizeVec[0] << " , " << sizeVec[1] << " , " << sizeVec[2] << endl;

					if(sizeVec[0] != 0 || sizeVec[1] != 0 || sizeVec[2] != 0)
					{
						vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
						cubeSource->SetXLength(sizeVec[0]);
						cubeSource->SetYLength(sizeVec[1]);
						cubeSource->SetZLength(sizeVec[2]);

						vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
						mapper->SetInputConnection(cubeSource->GetOutputPort());
						   
						vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
						actor->SetMapper(mapper);
						actor->SetUserTransform(visTransform);

						if(bHasMaterial)
						{
							string mtlname = obj->GetElement("visual")->GetElement("material")->GetElement("script")->GetElement("name")->Get<string>();

							cout << "mtlname: " << mtlname << endl;

							if(mtlname != "__default__")
							{
                                AddOgreMaterailToVTKActor(actor, mtlname, parentDir);
							}
							else
							{
								sdf::ElementPtr mtlPtr = obj->GetElement("visual")->GetElement("material");
								AddSdfMaterailToVTKActor(actor, mtlPtr);
							}
					
						}


						renderer->AddActor(actor);
						double *bounds = actor->GetBounds();	
						boundingbox.AddBounds(bounds);

					}

				}
			
				if(geo->HasElement("cylinder") == true)
				{
					sdf::ElementPtr cylinder = geo->GetElement("cylinder");
					double radius = cylinder->GetElement("radius")->Get<double>();
					double length = cylinder->GetElement("length")->Get<double>();
					
					vtkSmartPointer<vtkCylinderSource> cylinderSource =  vtkSmartPointer<vtkCylinderSource>::New();
					cylinderSource->SetCenter(0.0, 0.0, 0.0);
					cylinderSource->SetRadius(radius);
					cylinderSource->SetHeight(length);
					cylinderSource->SetResolution(100);  
					
					vtkSmartPointer<vtkTransform> cylinderTransform = vtkSmartPointer<vtkTransform>::New();
					//cylinderTransform->RotateX(3.1415926/2);
					cylinderTransform->RotateX(90);
					//cylinderTransform->RotateX(270);
					cylinderTransform->Update();

					vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
					transformFilter->SetTransform(cylinderTransform);
					transformFilter->SetInputConnection(cylinderSource->GetOutputPort());
					transformFilter->Update();
 
					//The triangulation has texture coordinates generated so we can map a texture onto it.
					vtkSmartPointer<vtkTextureMapToCylinder> tmapper = vtkSmartPointer<vtkTextureMapToCylinder>::New();
					//vtkSmartPointer<vtkTextureMapToSphere> tmapper = vtkSmartPointer<vtkTextureMapToSphere>::New();
					tmapper->SetInputConnection(transformFilter->GetOutputPort());
					tmapper->PreventSeamOn();

					//We scale the texture coordinate to get some repeat patterns.
					vtkSmartPointer<vtkTransformTextureCoords> xform = vtkSmartPointer<vtkTransformTextureCoords>::New();
					xform->SetInputConnection(tmapper->GetOutputPort());
					xform->SetScale(1, 1, 1);


					vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
					mapper->SetInputConnection(xform->GetOutputPort());
					   
					vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
					actor->SetMapper(mapper);
					actor->SetUserTransform(visTransform);

					if(bHasMaterial)
					{
						string mtlname = vis->GetElement("material")->GetElement("script")->GetElement("name")->Get<string>();

						cout << "mtlname: " << mtlname << endl;

						vector<string> uriVec;

						if(vis->GetElement("material")->GetElement("script")->HasElement("uri"))
						{
							for(sdf::ElementPtr uri = vis->GetElement("material")->GetElement("script")->GetElement("uri"); uri; uri = uri->GetNextElement("uri"))
							{
								uriVec.push_back(parentDir + uri->Get<std::string>());
								cout << "uri:  " << parentDir + uri->Get<std::string>() << endl;
							}
						}

						if(uriVec.size() == 2 && bHasBeerMaterial == false )
						{
							bHasBeerMaterial = true;

							std::string mtlfile = uriVec[0] + "/beer.material";
							
							std::ifstream sfile(mtlfile.c_str());
							Ogre::DataStreamPtr dataStream(OGRE_NEW Ogre::FileStreamDataStream(&sfile,false));  // setting "true" will cause free(): invalid pointer, Aborted (core dumped).

							scriptcompiler->compile(dataStream->getAsString(),"",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

						}


						if(mtlname != "__default__")
						{
                            AddOgreMaterailToVTKActor(actor, mtlname, parentDir);
						}
				
					}
					
					else
					{
						sdf::ElementPtr mtlPtr = obj->GetElement("visual")->GetElement("material");
						AddSdfMaterailToVTKActor(actor, mtlPtr);
					}


					renderer->AddActor(actor);
					double *bounds = actor->GetBounds();	
					boundingbox.AddBounds(bounds);


				}
			
				if(geo->HasElement("mesh") == true)
				{
					sdf::ElementPtr mesh = geo->GetElement("mesh");
				
					if(mesh->HasElement("uri") == false)
					{
						continue;
					}

					string meshfile = parentDir + mesh->GetElement("uri")->Get<std::string>();
					string meshMTLfile = meshfile + ".mtl";
					cout << "mesh file: " << meshfile << endl;
					cout << "mesh.mtl file: " << meshMTLfile << endl;

					// Read the mesh file and mtl file.
					vtkSmartPointer<vtkOBJImporter> importer = vtkSmartPointer<vtkOBJImporter>::New();
					importer->SetFileName(meshfile.data());
					importer->SetFileNameMTL(meshMTLfile.data());

					//--------------------------------------------------------------------------------//
					// Visualization
					//--------------------------------------------------------------------------------//
					vtkSmartPointer<vtkRenderer> tmp_render = vtkSmartPointer<vtkRenderer>::New();
					vtkSmartPointer<vtkRenderWindow> tmp_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

					tmp_renderWindow->AddRenderer(tmp_render);
					
					importer->SetRenderWindow(tmp_renderWindow);
					importer->Update();
				
					vtkSmartPointer<vtkTransform> meshTransform = vtkSmartPointer<vtkTransform>::New();
					meshTransform->RotateX(90);
				    meshTransform->PostMultiply();
				    meshTransform->Update();
   
					meshTransform->Concatenate(transform->GetMatrix());
				    meshTransform->Update();

					
					vtkSmartPointer<vtkActorCollection> actorCollection = tmp_render->GetActors();
					actorCollection->InitTraversal();
					for(vtkIdType i = 0; i < actorCollection->GetNumberOfItems(); i++)
					{
						vtkSmartPointer<vtkActor> nextActor = actorCollection->GetNextActor();
						nextActor->SetUserTransform(meshTransform);
						double *bounds = nextActor->GetBounds();	
						boundingbox.AddBounds(bounds);

						renderer->AddActor(nextActor);


						std::cout << "nextActor " << i << " : " << nextActor->GetClassName() << std::endl;
						if(nextActor->IsA("vtkCubeAxesActor"))
						{
							std::cout << "nextActor " << i << " is a vtkCubeAxesActor!" << std::endl;
						}
						else
						{
							std::cout << "nextActor " << i << " is NOT a vtkCubeAxesActor!" << std::endl;
						}
					}

				}
			
			}
            
		}

	}
		
	//------------------------------------//
	// Add the axe actor
	//------------------------------------//
	vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
	axes->AxisLabelsOff();
	axes->SetTotalLength(1,1,1);

	//renderer->AddActor(axes);

    
	//------------------------------------//
	// Add the outline bounding box actor.
    //------------------------------------//
    double bbox[6];
	boundingbox.GetBounds(bbox);
	for(int i=0; i<6; i++)
	{
		cout<<"i = " << i << " : " << setprecision(6) << bbox[i] << endl;
	}

	vtkSmartPointer<vtkActor> bboxActor = BoundingBoxVTKActor(bbox);
	//renderer->AddActor(bboxActor);

	//------------------------------------//
	// Set the camera.
	//------------------------------------//
	vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera(); 
	camera->SetPosition (5, 10, 10.0);
	camera->SetViewUp (-1, -5, -50.5);
	//camera->SetFocalPoint (5, 1, 0);
	camera->ParallelProjectionOn();
    camera->Zoom(0.75);
	
	renderer->ResetCameraClippingRange();
	//camera->ComputeViewtStates PlaneNormal();
	
	renderer->SetActiveCamera(camera);
	renderer->ResetCamera(); // ResetCamera() will change the camera center.
	
	

	//------------------------------------//
	// Visualization.
	//------------------------------------//
	//
	renderer->SetBackground(0.6, 0.6, 0.6); // Background color green


	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    if(bTakeRGBD == true)
	{
		renderWindow->SetOffScreenRendering( 1 ); 
	}
	renderWindow->AddRenderer(renderer);
	if(bTakeRGBD == true)
	{
		renderWindow->Render();
	}


	//------------------------------------//
	// Take a RGBD images or only show it.
	//------------------------------------//
	//
	if(bTakeRGBD == true)
	{
		// Save the RGB images.
		vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
		windowToImageFilter->SetInput(renderWindow);
		windowToImageFilter->SetMagnification(3); //image quality
		//windowToImageFilter->Update();

		vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
		writer->SetFileName("rgb_img.png");
		writer->SetInputConnection(windowToImageFilter->GetOutputPort());
		writer->Write();
		
		windowToImageFilter->SetInputBufferTypeToZBuffer();        //Extract z buffer value
		//windowToImageFilter->Update();

		// Create Depth Map
		vtkSmartPointer<vtkImageShiftScale> scale = vtkSmartPointer<vtkImageShiftScale>::New();
		scale->SetOutputScalarTypeToUnsignedChar();
		scale->SetInputConnection(windowToImageFilter->GetOutputPort());
		scale->SetShift(0);
		scale->SetScale(-255);
					 
		// Write depth map as a .bmp image
		vtkSmartPointer<vtkBMPWriter> imageWriter = vtkSmartPointer<vtkBMPWriter>::New();
		imageWriter->SetFileName("depth_img.bmp");
		imageWriter->SetInputConnection(scale->GetOutputPort());
		imageWriter->Write();

	}
	else
	{
		// Interactive visualization.	
		vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
		renderWindowInteractor->SetRenderWindow(renderWindow);
		
		renderWindowInteractor->Start();
	}



	return 0;
}



