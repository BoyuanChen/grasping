-- See: https://gist.github.com/walterlua/978161
-- Set up the table
table.copy = function( t, ... )
    local copyShallow = function( src, dst, dstStart )
        local result = dst or {}
        local resultStart = 0
        if dst and dstStart then
            resultStart = dstStart
        end
        local resultLen = 0
        if "table" == type( src ) then
            resultLen = #src
            for i=1,resultLen do
                local value = src[i]
                if nil ~= value then
                    result[i + resultStart] = value
                else
                    resultLen = i - 1
                    break;
                end
            end
        end
        return result,resultLen
    end

    local result, resultStart = copyShallow( t )

    local srcs = { ... }
    for i=1,#srcs do
        local _,len = copyShallow( srcs[i], result, resultStart )
        resultStart = resultStart + len
    end
    
    return result
end

---------------------------------------------------------------------
-- See: http://nocurve.com/2014/03/05/simple-csv-read-and-write-using-lua/
local function csvfile_split(str, sep)
    sep = sep or ','
    fields={}
    local matchfunc = string.gmatch(str, "([^"..sep.."]+)")
    if not matchfunc then return {str} end
    for str in matchfunc do
        table.insert(fields, str)
    end
    return fields
end

---------------------------------------------------------------------
-- See: http://nocurve.com/2014/03/05/simple-csv-read-and-write-using-lua/
function csvfile_read(path, sep, tonum)
    tonum = tonum or true
    sep = sep or ','
    local csvFile = {}
    local file = assert(io.open(path, "r"))
    for line in file:lines() do
        fields = csvfile_split(line, sep)
        if tonum then -- convert numeric fields to numbers
            for i=1,#fields do
                fields[i] = tonumber(fields[i]) or fields[i]
            end
        end
        table.insert(csvFile, fields)
    end
    file:close()
    return csvFile
end

---------------------------------------------------------------------
-- See: http://nocurve.com/2014/03/05/simple-csv-read-and-write-using-lua/
function csvfile_write(file, data, sep)

    sep = sep or ','
    
    for i=1,#data do
        file:write(data[i])
        file:write(sep)
    end
    file:write('\n')
    file:flush()
end

---------------------------------------------------------------------
-- See: http://nocurve.com/2014/03/05/simple-csv-read-and-write-using-lua/
function string:split( inSplitPattern, outResults )
 
   if not outResults then
      outResults = {}
   end
   local start = 1
   local theSplitStart, theSplitEnd = string.find(self, inSplitPattern, start)
   while theSplitStart do
      table.insert( outResults, string.sub( self, start, theSplitStart-1))
      start = theSplitEnd + 1
      theSplitStart, theSplitEnd = string.find( self, inSplitPattern, start)
   end
   table.insert( outResults, string.sub(self, start))
   return outResults
end


---------------------------------------------------------------------

-- Gets the bounding box components of an object
function getObjectBoxPositions(object_handle)

    local r, min_x = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_min_x)
    local r, max_x = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_max_x)
    local r, min_y = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_min_y)
    local r, max_y = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_max_y)
    local r, min_z = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_min_z)
    local r, max_z = simGetObjectFloatParameter(
		object_handle, sim_objfloatparam_objbbox_max_z)

    return {min_x, max_x, min_y, max_y, min_z, max_z}
end


-- objectBoxPositions = (min_x, max_x, min_y, max_y, min_z, max_z)
function getObjectBoxDimensions(objectBoxPositions)

    local dim ={
		math.abs(objectBoxPositions[2] - objectBoxPositions[1]),
        math.abs(objectBoxPositions[4] - objectBoxPositions[3]),
        math.abs(objectBoxPositions[5] - objectBoxPositions[6])}
    return dim
end


--We probably do not need this function.
function collectImages()

	-- Rotate the camera and take picture
	local cm = simGetObjectMatrix(camera_dummy, workspace_object)

	simSetObjectMatrix(depthCam, workspace_object, cm)
	local object_image = simGetVisionSensorDepthBuffer(depthCam)

	simSetObjectMatrix(colorCam, workspace_object, cm)
	local table_image = simGetVisionSensorDepthBuffer(colorCam)
	
	return object_image, table_image
	
end


--This function is really important to move the gripper
--Need to transfer the output of Graspit! into csv files, read them here and send them into this function
-- Moves the gripper by moving the palm-attached coordinate frame
function moveGripper(gripper, wrtHandleLocation, dummy_location)

	-- Move the gripper to a given location
    local cm = simGetObjectMatrix(dummy_location, wrtHandleLocation)
	simSetObjectMatrix(gripper, wrtHandleLocation, cm)

	--Move the gripper back some distance WRT palm-attached coordinate frame
	local modelFrameRelativeToDummy=simGetObjectPosition(gripper, gripper_palm)
    simSetObjectPosition(gripper,gripper,modelFrameRelativeToDummy)
	    
end


function parse_data(data)

	local names = {}
    local masses = {}
    local coms = {}
    local inertias = {}

	for i = 1, #data, 1 do

		local name = data[i][1]
        local mass = data[i][2]
        local com = {data[i][3], data[i][4], data[i][5]}
        local inertia = 
			{data[i][6], data[i][7], data[i][8],
             data[i][9], data[i][10], data[i][11],
             data[i][12], data[i][13], data[i][14]}

		table.insert(names, name)
        table.insert(masses, mass)
        table.insert(coms, com)
        table.insert(inertias, inertia)
	end
	
	return names, masses, coms, inertias

end


--We probably do not need this function but may need the part (move tht object to center of workspace) inside this function
threadFunction=function()

    -- Make the object dynamic and "fall" to a natural pose
    simSetObjectInt32Parameter(object, sim_shapeintparam_static, 0) 
    simResetDynamicObject(object)
	simSwitchThread()
	simWait(15)

	--for j = 1, 5, 1 do
	--	simAddForce(object, {0,0,0}, {20,0,0})
	--	simWait(1)
	--end

    -- make object static again 
    simSetObjectInt32Parameter(object, sim_shapeintparam_static, 1) 
    simResetDynamicObject(object)
    simSwitchThread()

    -- Move object to center of workspace (keep orientation)
    local obj_pos = simGetObjectPosition(object, -1)
	
    if obj_pos ~= nil then
                
        local wp = simGetObjectPosition(workspace_object, -1)
        if obj_pos[3] <= wp[3] then obj_pos[3] = obj_pos[3] + wp[3] end
        simSetObjectPosition(object, -1, {0, 0, obj_pos[3]})
        simSwitchThread()
					
		simSetObjectPosition(contact_dummy, object, {0,0,WORKSPACE_RADIUS/2})
		simSetObjectOrientation(contact_dummy, -1, simGetObjectOrientation(gripper_palm,-1))
		moveGripper(gripper_object, object, contact_dummy)
		simSwitchThread()
		
        local workspace2object = simGetObjectMatrix(object, workspace_object)   
        local object2gripper = simGetObjectMatrix(gripper_palm, object)
        local bbox = getObjectBoxPositions(object)
		local mass, inertia,com = simGetShapeMassAndInertia(object)

        csvfile_write(file, table.copy({OBJECT_NAME}, com, inertia, bbox,
                       workspace2object, object2gripper))
		
        simSwitchThread()
    end
	
	simAddDrawingObjectItem(sphereContainer,nil) -- reset
	simAddDrawingObjectItem(lineContainer,  nil) --reset
	
end





--local file_dir = 'C:\\Users\\Matt\\Documents\\grasping\\config.lua'
local file_dir = '/home/robot/Documents/grasping/lib/lua_config.lua'






package.path = package.path .. ';' .. file_dir
require 'config'
if myconf == nil then 
	print('UNABLE TO LOAD CONFIGURATION. CHECK FILE_DIR IS CORRECT.')
	simStopSimulation()
end

path_sep = myconf.path_sep
mesh_dir = myconf.mesh_dir
save_dir = myconf.data_dir
object_file = myconf.object_list_file
output_pose_file_name = myconf.output_pose_file_name

-- --------------- VISUALIZATION PROPERTIES -------------------------
black = {0, 0, 0}
purple = {1, 0, 1}
lightBlue = {0, 1, 1}
red = {1, 0, 0}

-- Used for plotting spherical and line-type objects
sphereContainer = simAddDrawingObject(
	sim_drawing_spherepoints, myconf.display_point_size, 
	myconf.display_point_density, -1, myconf.display_num_points,
	black, black, black, light_blue)
lineContainer = simAddDrawingObject(
	sim_drawing_lines, myconf.display_vector_width, 
	myconf.display_point_density, -1, myconf.display_num_points,
	black, black, black, purple)



-- --------------- SIMULATION SPECIFIC OBJECT HANDLES -------------------------

gripper_object = simGetObjectHandle('BarrettHand')
table_object = simGetObjectHandle('customizableTable_tableTop')
contact_dummy = simGetObjectHandle('contact_point') -- Where we will grasp from
gripper_palm = simGetObjectHandle('BarrettHand_PACF')


print('--------------------------------------------')
print('Using config: '..myconf.config)
print('Using path seperator: '..path_sep)
print('Using data directory: '..mesh_dir)
print('Using object file: '..object_file)
print('Saving data to dir: '..save_dir)
print('Saving pose data to: '..output_pose_file_name)
print('--------------------------------------------\n')

-- Object constants
OBJECT_START_POSITION = myconf.object_start_position 

-- Note: Flag combinations set the WHOLE mask, not just the specific portions
GRIPPER_PROP_STATIC = 
	sim_modelproperty_not_dynamic     + 
    sim_modelproperty_not_renderable  +
	sim_modelproperty_not_respondable  
OBJECT_PROP_RESPONDABLE = 
	sim_modelproperty_not_measurable +
	sim_modelproperty_not_renderable
GRIPPER_PROP_VISIBLE = 
	sim_modelproperty_not_renderable + 
	sim_modelproperty_not_measurable 
GRIPPER_PROP_INVISIBLE = 
	sim_modelproperty_not_collidable  + 
    sim_modelproperty_not_renderable  + 
    sim_modelproperty_not_visible     + 
    sim_modelproperty_not_respondable + 
    sim_modelproperty_not_dynamic

simSetModelProperty(gripper_object, GRIPPER_PROP_INVISIBLE)

local param_file = simGetStringParameter(sim_stringparam_app_arg1)

if param_file == '' then 
	input_file = mesh_dir .. object_file
else 
	input_file  = mesh_dir .. param_file
end

print('Loading simulation info ...')
local siminfo = csvfile_read(input_file, ',', true)

object_names, object_masses, object_coms, object_inertias = parse_data(siminfo)

-- --------------- RESET THE SIMULATION FOR EACH OBJECT IN FILE ---------------

local out_file = save_dir .. output_pose_file_name 
file = assert(io.open(out_file, 'w'))

workspace_object = simCreateDummy(0.01)
simSetObjectName(workspace_object,'workspace')

-- Loop over all the objects
for object_idx = 1, #object_names, 1 do
    


	-- Load the object mesh
    OBJECT_NAME = object_names[object_idx]
    MESH_FILE = mesh_dir .. path_sep .. OBJECT_NAME .. '.stl'
	
    print('OBJECT: '..object_idx..'/'..(#object_names))
	print('object name: '..OBJECT_NAME)

    mesh_vertices, indices, _, names = simImportMesh(4, MESH_FILE, 0, 0.001,1.0)


    -- Make sure the mesh has been loaded / object created
	if(mesh_vertices) then

        global_object_name = object_names[object_idx]
        global_object_mass = object_masses[object_idx]
        global_object_com  = object_coms[object_idx]
        global_object_inertia = object_inertias[object_idx]

		object = simCreateMeshShape(0,0,mesh_vertices[1],indices[1])
		simSetObjectName(object,'mesh_part')
        simSetShapeColor(object,nil,sim_colorcomponent_transparency,{0.5})
        
		-- Make sure we were successful in decomposing the object
		if object == nil then simStopSimulation() end
		
        -- NOTE: can turn on 'visualize inertias' by right-clicking on
        -- a dynamically enabled item within the simulator
        simSetShapeMassAndInertia(object, global_object_mass, global_object_inertia, global_object_com)
		simReorientShapeBoundingBox(object, -1)

		-- Set the object material (defines friction and contact)
		simSetShapeMaterial(object, simGetMaterialId(myconf.object_material))
		simSetObjectInt32Parameter(object, sim_shapeintparam_respondable, 1)
		simAddObjectToSelection({object})
		
		-- ----------------- CREATE A WORKSPACE REFERENCE ---------------------
		local bbox = getObjectBoxPositions(object)
		local dim = getObjectBoxDimensions(bbox)

		WORKSPACE_RADIUS = math.sqrt( dim[1]^2+dim[2]^2+dim[3]^2)
	
		local table_mtx = simGetObjectMatrix(table_object,-1)
		simSetObjectMatrix(workspace_object, -1, table_mtx)

		-- Set the generated part to be the object parent, and set starting position 
		simSetObjectPosition(object, workspace_object, OBJECT_START_POSITION)

		-- Here we execute the regular thread code:
		res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
		if not res then
			print(err)
			simAddStatusbarMessage('Lua runtime error: '..err)
		end

		simRemoveObject(object)

    end

end

file:close()

