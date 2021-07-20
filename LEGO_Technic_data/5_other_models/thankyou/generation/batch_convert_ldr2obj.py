import bpy
import os

for root, dirs, files in os.walk(".", topdown=False):
    for name in files:
        file_path = os.path.join(root, name)
        if file_path.endswith('.ldr'):
            bpy.ops.wm.read_homefile()
            print ('processing file ' + file_path + '\n') 
            # Import LDraw models (.mpd/.ldr/.l3b/.dat)
            bpy.ops.import_scene.importldraw(filepath = file_path, 
            filter_glob="*.mpd;*.ldr;*.l3b;*.dat", 
            ldrawPath="/Users/xuhao/tools/ldraw", 
            importScale=0.01, resPrims='High', 
            smoothParts=True, look='normal', 
            colourScheme='lgeo', addGaps=True, 
            gapsSize=0.005, curvedWalls=True, 
            importCameras=True, linkParts=True, 
            numberNodes=True, positionOnGround=True, 
            flatten=False, useUnofficialParts=True, 
            useLogoStuds=False, instanceStuds=False, 
            resolveNormals='guess', bevelEdges=True, 
            bevelWidth=0.5, addEnvironment=False, 
            positionCamera=False, cameraBorderPercentage=5)

            export_file_name = file_path + ".obj"
            # Save a Wavefront OBJ File
            bpy.ops.export_scene.obj(filepath = export_file_name, 
            check_existing=True, axis_forward='-Z', axis_up='Y', 
            filter_glob="*.obj;*.mtl", use_selection=False, 
            use_animation=False, use_mesh_modifiers=True, use_mesh_modifiers_render=False, 
            use_edges=True, use_smooth_groups=False, use_smooth_groups_bitflags=False, 
            use_normals=True, use_uvs=True, use_materials=True, use_triangles=False, 
            use_nurbs=False, use_vertex_groups=False, use_blen_objects=True, group_by_object=True, 
		    group_by_material=False, keep_vertex_order=False, global_scale=1, path_mode='AUTO')

            print ('file ' + export_file_name + 'generated\n')




