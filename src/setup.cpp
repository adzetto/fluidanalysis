#include "setup.hpp"



void main_setup() { // 3D model analysis for FluidX3D; required extensions in defines.hpp: VOLUME_FORCE, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 1.0f, 0.5f), 6000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution - increased to use more GPU memory
	const float lbm_Re = 10000.0f;
	const float lbm_u = 0.075f;
	const float si_u = 15.0f; // flow speed in m/s
	const float si_length = 0.2f; // characteristic length in m
	const float si_nu = 1.48E-5f; // kinematic viscosity in m^2/s (air at 20°C)
	const float si_rho = 1.225f; // density in kg/m^3 (air at 20°C)
	const float si_T = 3.0f; // simulation time in seconds - increased for better video quality
	
	// Set unit conversion between LBM and SI units
	units.set_m_kg_s((float)lbm_N.x, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	
	print_info("Re = " + to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));
	print_info(to_string(si_T, 3u) + " seconds = " + to_string(lbm_T) + " time steps");
	
	LBM lbm(lbm_N, lbm_nu);
	
	// ###################################################################################### define geometry ######################################################################################
	// Import 3D model from STL file in model folder
	const float3 center = lbm.center();
	const float size = 0.4f * (float)lbm_N.x; // Adjust size as needed
	
	// Create rotation matrix to ensure front of the car faces the X axis
	// Try different rotations to find the correct orientation
	// Option 1: Rotate 90 degrees around Z axis 
	//float3x3 rotation = float3x3(float3(0, 0, 1), radians(90.0f));
	// Option 2: Rotate 180 degrees around Z axis
	//float3x3 rotation = float3x3(float3(0, 0, 1), radians(180.0f));
	// Option 3: Rotate 90 degrees around Y axis to point car front towards X axis
	float3x3 rotation = float3x3(float3(0, 1, 0), radians(90.0f));
	// Option 4: No rotation, just try direct alignment
	//float3x3 rotation = float3x3(1.0f);
	
	print_info("Looking for STL file in model directory...");
	// Apply rotation to make sure front of car is facing the flow
	lbm.voxelize_stl("C:/Users/adzetto/Downloads/FluidX3D-master/model/model_3d.STL", center, rotation, size, TYPE_S|TYPE_X);
	
	// Set up the simulation domain
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();
	parallel_for(lbm.get_N(), [&](ulong n) { 
		uint x=0u, y=0u, z=0u; 
		lbm.coordinates(n, x, y, z);
		
		// Initialize flow field - flow from negative X direction (front of the car)
		if(lbm.flags[n]!=TYPE_S) lbm.u.x[n] = -lbm_u; // Negative X direction
		
		// Set boundary conditions
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E;
	});
	
	// ####################################################################### run simulation, export images and data ##########################################################################
	// Set visualization modes - change to show solid surface without overwhelming velocity field
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE; // Only show the solid surface
	
	// Create directory for video frames
	const string export_path = "export/";
	print_info("Video frames will be saved to: " + export_path);
	
	// Set camera parameters for a better view
	lbm.graphics.set_camera_free(center + float3(-0.3f*size, 0.1f*size, 0.1f*size), 0.0f, 0.0f, 90.0f);
	
	// Initialize simulation
	lbm.run(0u, lbm_T);
	lbm.write_status();
	
	// Print visualization mode instructions
	print_info("---------- Visualization Controls ----------");
	print_info("P: Start/pause simulation");
	print_info("1: Toggle solid surface visualization");
	print_info("2: Toggle velocity field visualization");
	print_info("3: Toggle streamlines visualization");
	print_info("4: Toggle vorticity visualization");
	print_info("H: Display help menu");
	print_info("-------------------------------------------");
	
	// Main simulation loop with camera animation
	const float video_length_seconds = 10.0f; // Target video length in seconds
	bool velocity_field_added = false;
	bool streamlines_added = false;
	bool vorticity_added = false;
	
	while(lbm.get_t() <= lbm_T) {
		// Calculate forces on the model
		const float3 force = lbm.object_force(TYPE_S|TYPE_X);
		const float drag = -force.x; // Negative sign because flow is in negative X direction
		const float lift = -force.z;
		
		// Convert forces to SI units and calculate coefficients
		const float si_drag = units.si_F(drag);
		const float si_lift = units.si_F(lift);
		const float area = sq(si_length); // Approximate reference area
		const float dynamic_pressure = 0.5f * si_rho * sq(si_u);
		const float cd = si_drag / (dynamic_pressure * area);
		const float cl = si_lift / (dynamic_pressure * area);
		
		// Print force information every 100 steps
		if(lbm.get_t() % 100 == 0) {
			print_info("Time step: " + to_string(lbm.get_t()) + 
				   ", Drag: " + to_string(si_drag, 3u) + " N" +
				   ", Lift: " + to_string(si_lift, 3u) + " N" + 
				   ", Cd: " + to_string(cd, 3u) + 
				   ", Cl: " + to_string(cl, 3u));
		}
		
		// Save video frames - use next_frame to determine when to save a new frame
		if(lbm.graphics.next_frame(lbm_T, video_length_seconds)) {
			// Calculate animation progress (0-1)
			const float t = (float)lbm.get_t()/(float)lbm_T;
			
			// Animate camera to circle around the model
			const float angle = t * 2.0f * 3.14159f; // Full 360-degree rotation
			const float radius = 0.6f * size;
			const float height = 0.2f * size + 0.1f * size * sin(angle * 2.0f); // Add some vertical movement
			
			// Update camera position to circle around the model
			const float3 camera_pos = center + float3(radius * sin(angle), radius * cos(angle), height);
			lbm.graphics.set_camera_free(camera_pos, degrees(angle) + 90.0f, 20.0f + 10.0f * sin(angle), 90.0f);
			
			// Add different visualization elements at different stages
			if(t > 0.2f && !velocity_field_added) {
				lbm.graphics.visualization_modes |= VIS_FIELD;
				velocity_field_added = true;
				print_info("Added velocity field visualization");
			}
			
			if(t > 0.4f && !streamlines_added) {
				lbm.graphics.visualization_modes |= VIS_STREAMLINES;
				streamlines_added = true;
				print_info("Added streamlines visualization");
			}
			
			if(t > 0.6f && !vorticity_added) {
				lbm.graphics.visualization_modes |= VIS_Q_CRITERION;
				vorticity_added = true;
				print_info("Added vorticity visualization");
			}
			
			// Save the frame to disk
			lbm.graphics.write_frame_png(export_path, false);
		}
		
		// Run simulation for one time step
		lbm.run(1u, lbm_T);
	}
	
	// Print instructions for converting frames to video
	print_info("Simulation complete. To convert frames to video, use FFmpeg:");
	print_info("ffmpeg -framerate 60 -pattern_type glob -i \"export/*/image-*.png\" -c:v libx264 -pix_fmt yuv420p -b:v 24M \"car_aerodynamics.mp4\"");
	
	// Final status
	lbm.write_status();
}

#ifdef BENCHMARK
#include "info.hpp"
void main_setup() { // benchmark; required extensions in defines.hpp: BENCHMARK, optionally FP16S or FP16C
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	uint mlups = 0u; {

		//LBM lbm( 32u,  32u,  32u, 1.0f);
		//LBM lbm( 64u,  64u,  64u, 1.0f);
		//LBM lbm(128u, 128u, 128u, 1.0f);
		LBM lbm(256u, 256u, 256u, 1.0f); // default
		//LBM lbm(384u, 384u, 384u, 1.0f);
		//LBM lbm(512u, 512u, 512u, 1.0f);

		//const uint memory = 1488u; // memory occupation in MB (for multi-GPU benchmarks: make this close to as large as the GPU's VRAM capacity)
		//const uint3 lbm_N = (resolution(float3(1.0f, 1.0f, 1.0f), memory)/4u)*4u; // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
		//LBM lbm(1u*lbm_N.x, 1u*lbm_N.y, 1u*lbm_N.z, 1u, 1u, 1u, 1.0f); // 1 GPU
		//LBM lbm(2u*lbm_N.x, 1u*lbm_N.y, 1u*lbm_N.z, 2u, 1u, 1u, 1.0f); // 2 GPUs
		//LBM lbm(2u*lbm_N.x, 2u*lbm_N.y, 1u*lbm_N.z, 2u, 2u, 1u, 1.0f); // 4 GPUs
		//LBM lbm(2u*lbm_N.x, 2u*lbm_N.y, 2u*lbm_N.z, 2u, 2u, 2u, 1.0f); // 8 GPUs

		// #########################################################################################################################################################################################
		for(uint i=0u; i<1000u; i++) {
			lbm.run(10u, 1000u*10u);
			mlups = max(mlups, to_uint((double)lbm.get_N()*1E-6/info.runtime_lbm_timestep_smooth));
		}
	} // make lbm object go out of scope to free its memory
	print_info("Peak MLUPs/s = "+to_string(mlups));
#if defined(_WIN32)
	wait();
#endif // Windows
} /**/
#endif // BENCHMARK