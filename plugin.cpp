#include "deformer.h"
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "Conar&Co", "1.0", "Any");
    /*
    MFnPlugin plugin(obj, "MikrosImage", PLUGIN_VERSION, "Any");

	MString info = "dummyDeformer v";
	info += PLUGIN_VERSION;
	info += " Mikros built (";
	info += __DATE__;
	info += ")";
	MGlobal::displayInfo(info);
	*/
#ifdef USE_GLDRAWER
	int result;
	status = MGlobal::executeCommand("pluginInfo -query -loaded \"glDrawer\"", result, false, false);
	if(status == MS::kSuccess)
	{
		if(result != 1)
		{
			status = MGlobal::executeCommand("loadPlugin \"glDrawer\"", false, false);
			if(status != MS::kSuccess)
			{
				MGlobal::displayError("Can NOT Load GLDrawer Plugin!");
				return status;
			}
		}
	}
	else
	{
		MGlobal::displayError("Can NOT Query GLDrawer Plugin!");
		return status;
	}
#endif

	if (!status) {
		status.perror("register CustomLocator Node");
		return status;
	}

	// register deformer node:
	status = plugin.registerNode(dummyDeformer::typeName,
    							dummyDeformer::typeId,
								dummyDeformer::creator,
								dummyDeformer::initialize,
								MPxNode::kDeformerNode);

    if (!status) {
    	status.perror("register Dummy Deformer Node");
		return status;
	}

    /*
    // make weights paintable:
    MGlobal::executeCommand( "makePaintable -attrType multiFloat -sm deformer "
    								+ collideDeformer::typeName + " weights;" );
	*/

    return status;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus result;
    MFnPlugin plugin(obj);

	result = plugin.deregisterNode( dummyDeformer::typeId );
	if (!result) {
		result.perror("deregister Dummy Deformer Node");
		return result;
	}

    return result;
}

