extends Node3D

enum State {
    FALLING,
    RISING
}

@export_group("Float Settings")
@export var buoyancy_offset: float = 0.1 # How high above water surface to float
@export var float_zone_above_water: float = 0.2 # Distance above water to enable floating mode
@export var float_zone_below_water: float = 1.5 # Distance below water to enable floating mode
@export var underwater_threshold: float = 0.125 # How far underwater before switching to rising
@export var above_water_threshold: float = -0.05 # How far above target before switching to falling

@export_group("Movement Forces")
@export var normal_fall_force: float = 50.0
@export var normal_rise_force: float = 30.0
@export var float_fall_force: float = 8.0 # Much gentler when floating
@export var float_rise_force: float = 12.0 # Much gentler when floating

@export_group("Speed Limits")
@export var normal_max_fall_speed: float = 50.0
@export var normal_max_rise_speed: float = 15.0
@export var float_max_fall_speed: float = 2.0 # Much slower when floating
@export var float_max_rise_speed: float = 3.0 # Much slower when floating

@export_group("Damping")
@export var normal_damping: float = 0.99
@export var float_damping: float = 0.2 # More damping when floating
@export var impact_speed_reduction: float = 0.3

@export_group("Advanced")
@export var sky_gravity_multiplier: float = 3.0 # Extra gravity when high above water
@export var sky_threshold: float = 10.0 # Height above water considered "sky"

@export_group("Center of Mass & Rotation")
@export var enable_com_rotation: bool = true # Enable center of mass rotation when floating
@export var rotation_strength: float = 2.0 # How strong the rotation force is
@export var rotation_damping: float = 0.95 # Damping for rotational movement
@export var com_offset: Vector3 = Vector3.ZERO # Manual offset for center of mass
@export var auto_calculate_com: bool = true # Automatically calculate COM from mesh
@export var buoyancy_up_vector: Vector3 = Vector3.UP # Which direction is "up" for buoyancy

@export_group("Transition Rotation")
@export var transition_time_threshold: float = 2.0 # Time after floating state change for enhanced rotation
@export var transition_rotation_multiplier: float = 128.0 # How much faster rotation is during transition
@export var transition_damping_multiplier: float = 0.6 # Additional damping during transition
@export var initial_impact_multiplier: float = 1600.0 # Extra strong rotation for first few frames of impact
@export var initial_impact_duration: float = 0.05 # How long the initial impact boost lasts
@export var snap_rotation_threshold: float = 0.2 # If rotation needed is small, snap instantly
@export var force_upright_on_impact: bool = true # Force logs to go upright immediately on water impact
@export var debug_rotation: bool = false # Print rotation debug info

@export var mesh_instance: MeshInstance3D # Reference to MeshInstance3D for center of mass calculation

const WATER_LEVEL: float = 0.0
var current_state: State = State.FALLING
var current_velocity: Vector3 = Vector3.ZERO
var target_y: float = 0.5 # Water level + buoyancy offset
var rigid_body: RigidBody3D
var is_floating: bool = false
var previous_floating_state: bool = false
var just_hit_water: bool = false
var previous_y_position: float = 0.0

# Center of mass and rotation variables
var center_of_mass: Vector3 = Vector3.ZERO
var current_angular_velocity: Vector3 = Vector3.ZERO

# Transition timing variables
var floating_state_change_time: float = 0.0
var time_since_last_change: float = 0.0
var is_in_transition: bool = false

func _ready() -> void:
    assert(mesh_instance is MeshInstance3D, "MeshInstance3D must be set for center of mass calculation")
    target_y = WATER_LEVEL + buoyancy_offset
    
    # Try to find RigidBody3D parent or child
    rigid_body = get_parent() as RigidBody3D
    if not rigid_body:
        rigid_body = get_node_or_null("RigidBody3D") as RigidBody3D
    
    if not rigid_body:
        print("Warning: No RigidBody3D found. Floaty script will move the Node3D directly.")
    
    # Calculate initial center of mass
    _calculate_center_of_mass()
    
    # Initialize floating state tracking
    previous_floating_state = is_floating
    previous_y_position = global_position.y

func _calculate_center_of_mass() -> void:
    """Calculate center of mass from mesh or use manual offset"""
    if auto_calculate_com and mesh_instance and mesh_instance.mesh:
        # Use AABB center as approximation for center of mass
        var aabb = mesh_instance.mesh.get_aabb()
        center_of_mass = aabb.get_center()
        
        # Apply mesh instance transform since it's a child of this Node3D
        center_of_mass = mesh_instance.transform * center_of_mass
    else:
        # Use manual offset
        center_of_mass = com_offset
    
    print("Center of mass calculated at: ", center_of_mass)

func _physics_process(delta: float) -> void:
    _check_water_impact()
    _update_floating_state()
    _update_transition_timing(delta)
    _update_movement_state()
    
    if rigid_body:
        _apply_physics_to_rigidbody(delta)
    else:
        _apply_physics_to_node(delta)
    
    # Apply center of mass rotation when floating OR during transition period
    if enable_com_rotation and (is_floating or is_in_transition):
        if rigid_body:
            _apply_com_rotation_rigidbody(delta)
        else:
            _apply_com_rotation_node(delta)
    
    # Update previous position for next frame
    previous_y_position = global_position.y

func _check_water_impact() -> void:
    """Check if object just hit the water surface"""
    var current_y = global_position.y
    
    # Check if we crossed the water level from above
    if previous_y_position > WATER_LEVEL and current_y <= WATER_LEVEL:
        just_hit_water = true
        _on_water_impact()
    else:
        just_hit_water = false

func _on_water_impact() -> void:
    """Called when object hits water surface"""
    floating_state_change_time = 0.0
    time_since_last_change = 0.0
    is_in_transition = true
    print("Object hit water surface - starting enhanced rotation")
    
    # Force immediate upright orientation if enabled
    if force_upright_on_impact:
        _force_upright_orientation()

func _force_upright_orientation() -> void:
    """Force the object to orient correctly immediately"""
    # Get world position of center of mass
    var world_com = global_transform * center_of_mass
    
    # Calculate desired up direction (towards water surface from COM)
    var to_surface = Vector3(world_com.x, WATER_LEVEL, world_com.z) - world_com
    var desired_up = buoyancy_up_vector
    
    # For logs, we want them to float horizontally, so use the shortest rotation
    var current_up = global_transform.basis.y
    var current_forward = global_transform.basis.z
    
    # Find the best orientation (either current up or forward should point up)
    var up_to_up_angle = current_up.angle_to(desired_up)
    var forward_to_up_angle = current_forward.angle_to(desired_up)
    var backward_to_up_angle = (-current_forward).angle_to(desired_up)
    
    var target_basis = global_transform.basis
    
    # Choose the orientation that requires the least rotation
    if up_to_up_angle < forward_to_up_angle and up_to_up_angle < backward_to_up_angle:
        # Current up is closest to desired up
        target_basis = target_basis.looking_at(global_position + current_forward, desired_up)
    elif forward_to_up_angle < backward_to_up_angle:
        # Current forward should become up
        target_basis = target_basis.looking_at(global_position + current_up, current_forward)
    else:
        # Current backward should become up
        target_basis = target_basis.looking_at(global_position + current_up, -current_forward)
    
    # Apply the new orientation
    global_transform.basis = target_basis
    
    # Stop any angular velocity
    if rigid_body:
        rigid_body.angular_velocity = Vector3.ZERO
    else:
        current_angular_velocity = Vector3.ZERO
    
    if debug_rotation:
        print("Forced upright orientation - angles: up=", up_to_up_angle, " forward=", forward_to_up_angle, " backward=", backward_to_up_angle)

func _update_floating_state() -> void:
    """Update whether we're in floating mode based on distance from water"""
    var current_y = global_position.y
    var distance_from_water = current_y - WATER_LEVEL
    
    # Store previous state
    previous_floating_state = is_floating
    
    # Check if we're within the floating zone
    if distance_from_water >= 0:
        # Above water - check against above water threshold
        is_floating = distance_from_water <= float_zone_above_water
    else:
        # Below water - check against below water threshold
        is_floating = abs(distance_from_water) <= float_zone_below_water
    
    # Check if floating state has changed
    if is_floating != previous_floating_state:
        _on_floating_state_changed()

func _on_floating_state_changed() -> void:
    """Called when the floating state changes"""
    floating_state_change_time = 0.0
    time_since_last_change = 0.0
    is_in_transition = true
    
    # Debug output
    if is_floating:
        print("Object entered floating mode")
    else:
        print("Object left floating mode")

func _update_transition_timing(delta: float) -> void:
    """Update transition timing variables"""
    if is_in_transition:
        time_since_last_change += delta
        
        # Check if we're still within the transition threshold
        if time_since_last_change >= transition_time_threshold:
            is_in_transition = false

func _get_rotation_multiplier() -> float:
    """Get the current rotation strength multiplier based on transition state"""
    if is_in_transition:
        var transition_progress = time_since_last_change / transition_time_threshold
        
        # Apply extra strong rotation for initial impact
        if time_since_last_change <= initial_impact_duration:
            var impact_progress = time_since_last_change / initial_impact_duration
            # Lerp from initial impact multiplier to transition multiplier
            var current_multiplier = lerp(initial_impact_multiplier, transition_rotation_multiplier, impact_progress)
            return current_multiplier
        else:
            # After initial impact, lerp from transition multiplier to normal
            var adjusted_progress = (time_since_last_change - initial_impact_duration) / (transition_time_threshold - initial_impact_duration)
            return lerp(transition_rotation_multiplier, 1.0, adjusted_progress)
    else:
        return 1.0

func _get_rotation_damping_multiplier() -> float:
    """Get the current rotation damping multiplier based on transition state"""
    if is_in_transition:
        var transition_progress = time_since_last_change / transition_time_threshold
        
        # Apply stronger damping for initial impact to prevent wild spinning
        if time_since_last_change <= initial_impact_duration:
            var impact_progress = time_since_last_change / initial_impact_duration
            # Start with stronger damping during initial impact, then ease to transition damping
            var initial_damping = transition_damping_multiplier * 0.4 # Much stronger damping initially
            var current_damping = lerp(initial_damping, transition_damping_multiplier, impact_progress)
            return current_damping
        else:
            # After initial impact, lerp from transition damping to normal
            var adjusted_progress = (time_since_last_change - initial_impact_duration) / (transition_time_threshold - initial_impact_duration)
            return lerp(transition_damping_multiplier, 1.0, adjusted_progress)
    else:
        return 1.0

func _update_movement_state() -> void:
    """Update whether we should be falling or rising based on thresholds"""
    var current_y = global_position.y
    
    match current_state:
        State.FALLING:
            # Switch to rising if we've gone far enough underwater
            var underwater_target = WATER_LEVEL - underwater_threshold
            if current_y <= underwater_target:
                current_state = State.RISING
        
        State.RISING:
            # Switch to falling if we've gone far enough above target
            var above_target = target_y + above_water_threshold
            if current_y >= above_target:
                current_state = State.FALLING

func _get_current_forces() -> Dictionary:
    """Get the appropriate forces based on current state and floating mode"""
    var forces = {}
    
    if is_floating:
        forces.fall_force = float_fall_force
        forces.rise_force = float_rise_force
        forces.max_fall_speed = float_max_fall_speed
        forces.max_rise_speed = float_max_rise_speed
        forces.damping = float_damping
    else:
        forces.fall_force = normal_fall_force
        forces.rise_force = normal_rise_force
        forces.max_fall_speed = normal_max_fall_speed
        forces.max_rise_speed = normal_max_rise_speed
        forces.damping = normal_damping
    
    return forces

func _is_sky_falling() -> bool:
    return global_position.y > WATER_LEVEL + sky_threshold

func _apply_com_rotation_rigidbody(delta: float) -> void:
    """Apply center of mass rotation to RigidBody3D"""
    # Get world position of center of mass
    var world_com = global_transform * center_of_mass
    
    # Calculate buoyancy force direction (towards water surface)
    var to_surface = Vector3(world_com.x, WATER_LEVEL, world_com.z) - world_com
    var buoyancy_direction = to_surface.normalized()
    
    # Calculate desired up direction (opposite to buoyancy force)
    var desired_up = - buoyancy_direction
    if desired_up.length() < 0.1:
        desired_up = buoyancy_up_vector
    
    # Get current up direction
    var current_up = global_transform.basis.y
    
    # Calculate rotation needed
    var rotation_axis = current_up.cross(desired_up)
    if rotation_axis.length() > 0.001:
        rotation_axis = rotation_axis.normalized()
        var rotation_angle = current_up.angle_to(desired_up)
        
        if debug_rotation:
            print("Rotation needed: ", rotation_angle, " radians (", rad_to_deg(rotation_angle), " degrees)")
        
        # Check for snap rotation during initial impact
        if is_in_transition and time_since_last_change <= initial_impact_duration:
            if rotation_angle < snap_rotation_threshold:
                # Snap instantly to correct orientation
                var snap_quaternion = Quaternion(current_up, desired_up)
                global_transform.basis = Basis(snap_quaternion) * global_transform.basis
                rigid_body.angular_velocity = Vector3.ZERO
                if debug_rotation:
                    print("Snapped to correct orientation")
                return
        
        # Apply transition multiplier to rotation strength
        var effective_rotation_strength = rotation_strength * _get_rotation_multiplier()
        
        if debug_rotation and is_in_transition:
            print("Effective rotation strength: ", effective_rotation_strength, " (multiplier: ", _get_rotation_multiplier(), ")")
        
        # Apply torque to rotate towards desired orientation
        var torque = rotation_axis * rotation_angle * effective_rotation_strength * rigid_body.mass
        rigid_body.apply_torque(torque)
    
    # Apply rotational damping with transition multiplier
    var effective_damping = rotation_damping * _get_rotation_damping_multiplier()
    rigid_body.angular_velocity *= effective_damping

func _apply_com_rotation_node(delta: float) -> void:
    """Apply center of mass rotation to Node3D directly"""
    # Get world position of center of mass
    var world_com = global_transform * center_of_mass
    
    # Calculate buoyancy force direction (towards water surface)
    var to_surface = Vector3(world_com.x, WATER_LEVEL, world_com.z) - world_com
    var buoyancy_direction = to_surface.normalized()
    
    # Calculate desired up direction (opposite to buoyancy force)
    var desired_up = - buoyancy_direction
    if desired_up.length() < 0.1:
        desired_up = buoyancy_up_vector
    
    # Get current up direction
    var current_up = global_transform.basis.y
    
    # Calculate rotation needed
    var rotation_axis = current_up.cross(desired_up)
    if rotation_axis.length() > 0.001:
        rotation_axis = rotation_axis.normalized()
        var rotation_angle = current_up.angle_to(desired_up)
        
        # Check for snap rotation during initial impact
        if is_in_transition and time_since_last_change <= initial_impact_duration:
            if rotation_angle < snap_rotation_threshold:
                # Snap instantly to correct orientation
                var snap_quaternion = Quaternion(current_up, desired_up)
                global_transform.basis = Basis(snap_quaternion) * global_transform.basis
                current_angular_velocity = Vector3.ZERO
                return
        
        # Apply transition multiplier to rotation strength
        var effective_rotation_strength = rotation_strength * _get_rotation_multiplier()
        
        # Update angular velocity
        current_angular_velocity += rotation_axis * rotation_angle * effective_rotation_strength * delta
        
        # Apply rotational damping with transition multiplier
        var effective_damping = rotation_damping * _get_rotation_damping_multiplier()
        current_angular_velocity *= effective_damping
        
        # Apply rotation
        if current_angular_velocity.length() > 0.001:
            var rotation_quaternion = Quaternion(current_angular_velocity.normalized(),
                                               current_angular_velocity.length() * delta)
            global_transform.basis = Basis(rotation_quaternion) * global_transform.basis

func _apply_physics_to_rigidbody(delta: float) -> void:
    var forces = _get_current_forces()
    var current_velocity_y = rigid_body.linear_velocity.y
    
    match current_state:
        State.FALLING:
            # Apply downward force
            var fall_force = forces.fall_force
            if _is_sky_falling() and not is_floating:
                fall_force *= sky_gravity_multiplier
            
            var gravity_vec = Vector3(0, -fall_force * rigid_body.mass, 0)
            rigid_body.apply_central_force(gravity_vec)
            
            # Limit fall speed
            if current_velocity_y < -forces.max_fall_speed:
                rigid_body.linear_velocity.y = - forces.max_fall_speed
        
        State.RISING:
            # Apply upward force
            var rise_strength = forces.rise_force * rigid_body.mass
            rigid_body.apply_central_force(Vector3(0, rise_strength, 0))
            
            # Limit rise speed
            if current_velocity_y > forces.max_rise_speed:
                rigid_body.linear_velocity.y = forces.max_rise_speed
    
    # Handle impact when hitting water from above
    if not is_floating and current_velocity_y < -10.0 and global_position.y <= WATER_LEVEL:
        rigid_body.linear_velocity.y *= impact_speed_reduction
    
    # Apply damping
    rigid_body.linear_velocity *= Vector3(forces.damping, forces.damping, forces.damping)

func _apply_physics_to_node(delta: float) -> void:
    var forces = _get_current_forces()
    
    match current_state:
        State.FALLING:
            # Apply downward acceleration
            var fall_force = forces.fall_force
            if _is_sky_falling() and not is_floating:
                fall_force *= sky_gravity_multiplier
            
            current_velocity.y -= fall_force * delta
            
            # Limit fall speed
            if current_velocity.y < -forces.max_fall_speed:
                current_velocity.y = - forces.max_fall_speed
        
        State.RISING:
            # Apply upward acceleration
            current_velocity.y += forces.rise_force * delta
            
            # Limit rise speed
            if current_velocity.y > forces.max_rise_speed:
                current_velocity.y = forces.max_rise_speed
    
    # Handle impact when hitting water from above
    if not is_floating and current_velocity.y < -10.0 and global_position.y <= WATER_LEVEL:
        current_velocity.y *= impact_speed_reduction
    
    # Apply damping
    current_velocity *= Vector3(forces.damping, forces.damping, forces.damping)
    
    # Apply movement
    global_position += current_velocity * delta

# Helper functions
func get_is_floating() -> bool:
    """Check if object is currently in floating mode"""
    return is_floating

func get_is_falling() -> bool:
    """Check if object is currently falling"""
    return current_state == State.FALLING

func get_is_rising() -> bool:
    """Check if object is currently rising"""
    return current_state == State.RISING

func get_is_in_transition() -> bool:
    """Check if object is currently in transition period"""
    return is_in_transition

func get_time_since_transition() -> float:
    """Get time elapsed since last floating state change"""
    return time_since_last_change

func get_current_state_name() -> String:
    """Get current state as string for debugging"""
    var state_name = ""
    match current_state:
        State.FALLING:
            state_name = "FALLING"
        State.RISING:
            state_name = "RISING"
        _:
            state_name = "UNKNOWN"
    
    if is_floating:
        state_name += " (floating)"
    
    if is_in_transition:
        state_name += " [TRANSITION]"
    
    return state_name

func set_buoyancy_height(height: float) -> void:
    """Set how high above water to float"""
    buoyancy_offset = height
    target_y = WATER_LEVEL + buoyancy_offset

func set_float_zones(above_water: float, below_water: float) -> void:
    """Set the floating zone thresholds separately"""
    float_zone_above_water = above_water
    float_zone_below_water = below_water

func force_state(new_state: State) -> void:
    """Force a specific state (for debugging/testing)"""
    current_state = new_state

func teleport_above_water(height: float) -> void:
    """Teleport object to height above water and start falling"""
    global_position.y = WATER_LEVEL + height
    current_state = State.FALLING
    if rigid_body:
        rigid_body.linear_velocity = Vector3.ZERO
        rigid_body.angular_velocity = Vector3.ZERO
    else:
        current_velocity = Vector3.ZERO
        current_angular_velocity = Vector3.ZERO
    
    # Reset transition state
    is_in_transition = false
    time_since_last_change = 0.0

# Center of mass functions
func get_center_of_mass() -> Vector3:
    """Get the current center of mass"""
    return center_of_mass

func set_center_of_mass(new_com: Vector3) -> void:
    """Manually set the center of mass"""
    center_of_mass = new_com
    auto_calculate_com = false

func recalculate_center_of_mass() -> void:
    """Force recalculation of center of mass"""
    _calculate_center_of_mass()

func set_rotation_strength(strength: float) -> void:
    """Set how strong the rotation correction is"""
    rotation_strength = strength

func enable_rotation(enabled: bool) -> void:
    """Enable or disable center of mass rotation"""
    enable_com_rotation = enabled

# Transition control functions
func set_transition_threshold(time: float) -> void:
    """Set how long the transition period lasts"""
    transition_time_threshold = time

func set_transition_rotation_multiplier(multiplier: float) -> void:
    """Set how much faster rotation is during transition"""
    transition_rotation_multiplier = multiplier

func set_transition_damping_multiplier(multiplier: float) -> void:
    """Set additional damping during transition"""
    transition_damping_multiplier = multiplier

func set_initial_impact_settings(multiplier: float, duration: float) -> void:
    """Set initial impact rotation strength and duration"""
    initial_impact_multiplier = multiplier
    initial_impact_duration = duration

func force_transition_state(in_transition: bool) -> void:
    """Force transition state for testing"""
    is_in_transition = in_transition
    if in_transition:
        time_since_last_change = 0.0
    else:
        time_since_last_change = transition_time_threshold