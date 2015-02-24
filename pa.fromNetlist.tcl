
# PlanAhead Launch Script for Post-Synthesis floorplanning, created by Project Navigator

create_project -name MyAmbder -dir "/home/martin/minispartan6/MyAmbder/planAhead_run_1" -part xc6slx9ftg256-3
set_property design_mode GateLvl [get_property srcset [current_run -impl]]
set_property edif_top_file "/home/martin/minispartan6/MyAmbder/system.ngc" [ get_property srcset [ current_run ] ]
add_files -norecurse { {/home/martin/minispartan6/MyAmbder} }
set_property target_constrs_file "MyAmber.ucf" [current_fileset -constrset]
add_files [list {MyAmber.ucf}] -fileset [get_property constrset [current_run]]
link_design
