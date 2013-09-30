#include "primitive.h"

///@file igl/primitive.cpp Primitives. @ingroup igl

/// Builds an animated transformation matrix
/// @param transformed Data structure describing transformations
/// @param time The time at which to compute the animated transformation
/// @return m The animated transformation matrix
mat4f transformed_matrix(TransformedSurface* transformed, float time) {
    auto m = identity_mat4f;
    vec3f trans, rot, scale = zero3f;

    // Null checks
    // If any of the transformed values are NULL, just use the normal translation member
    if(transformed->anim_translation != NULL)
        trans = transformed->translation + keyframed_value(transformed->anim_translation, time);
    else
        trans = transformed->translation;
    if(transformed->anim_rotation_euler != NULL)
        rot = transformed->rotation_euler + keyframed_value(transformed->anim_rotation_euler, time);
    else
        rot = transformed->rotation_euler;
    if(transformed->anim_scale != NULL)
        scale = transformed->scale * keyframed_value(transformed->anim_scale, time);
    else
        scale = transformed->scale;

    // Translation
    m *= mat4f(1, 0, 0, trans.x,
               0, 1, 0, trans.y,
               0, 0, 1, trans.z,
               0, 0, 0 , 1);

    // Rotate around z-axis
    auto rot_z = rot.z;
    m *= mat4f(cos(rot_z), -sin(rot_z), 0, 0,
               sin(rot_z), cos(rot_z), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1);

    // Rotate around y-axis
    auto rot_y = rot.y;
    m *= mat4f(cos(rot_y), 0, sin(rot_y), 0,
               0, 1, 0, 0,
               -sin(rot_y), 0, cos(rot_y), 0,
               0, 0, 0, 1);

    // Rotate around x-axis
    auto rot_x = rot.x;
    m *= mat4f(1, 0, 0, 0,
               0, cos(rot_x), -sin(rot_x), 0,
               0, sin(rot_x), cos(rot_x), 0,
               0, 0, 0, 1);

    // Scaling
    m *= mat4f(scale.x, 0, 0, 0,
               0, scale.y, 0, 0,
               0, 0, scale.z, 0,
               0, 0, 0, 1);

    // Return the transformation matrix
    return m;
}

/// Builds an inverse animated transformation matrix
/// @param transformed Data structure describing transformations
/// @param time The time at which to compute the animated transformation
/// @return m The animated transformation matrix
mat4f transformed_matrix_inv(TransformedSurface* transformed, float time) {
    auto m = identity_mat4f;
    vec3f trans, rot, scale = zero3f;

    // Null checks
    // If any of the transformed values are NULL, just use the normal translation member
    if(transformed->anim_translation != NULL)
        trans = transformed->translation + keyframed_value(transformed->anim_translation, time);
    else
        trans = transformed->translation;
    if(transformed->anim_rotation_euler != NULL)
        rot = transformed->rotation_euler + keyframed_value(transformed->anim_rotation_euler, time);
    else
        rot = transformed->rotation_euler;
    if(transformed->anim_scale != NULL)
        scale = transformed->scale * keyframed_value(transformed->anim_scale, time);
    else
        scale = transformed->scale;

    // Scaling
    m *= mat4f(1/scale.x, 0, 0, 0,
               0, 1/scale.y, 0, 0,
               0, 0, 1/scale.z, 0,
               0, 0, 0, 1);

    // Rotate around x-axis
    auto rot_x = rot.x;
    rot_x *= -1;
    m *= mat4f(1, 0, 0, 0,
               0, cos(rot_x), -sin(rot_x), 0,
               0, sin(rot_x), cos(rot_x), 0,
               0, 0, 0, 1);

    // Rotate around y-axis
    auto rot_y = rot.y;
    rot_y *= -1;
    m *= mat4f(cos(rot_y), 0, sin(rot_y), 0,
               0, 1, 0, 0,
               -sin(rot_y), 0, cos(rot_y), 0,
               0, 0, 0, 1);

    // Rotate around z-axis
    auto rot_z = rot.z;
    rot_z *= -1;
    m *= mat4f(cos(rot_z), -sin(rot_z), 0, 0,
               sin(rot_z), cos(rot_z), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1);

    // Translation
    m *= mat4f(1, 0, 0, -trans.x,
               0, 1, 0, -trans.y,
               0, 0, 1, -trans.z,
               0, 0, 0 , 1);

    // Return the inverse transformation matrix
    return m;
}

/// Computes the pose and rest frames at time for each bone
/// @param skinned Where rest bone data (wrt to parent bone's frame) and the pose rotation data are stored
/// @param time The time at which to compute each bone's pose frames
/// @param pose_frames The frame of each bone in pose position wrt the frame of root bone (object frame)
/// @param rest_frames The frame of each bone in rest position wrt the frame of root bone (object frame)
void skinned_bone_frames(SkinnedSurface* skinned, float time, vector<frame3f>& pose_frames, vector<frame3f>& rest_frames) {
    pose_frames.resize(skinned->bones.size(), identity_frame3f);
    rest_frames.resize(skinned->bones.size(), identity_frame3f);

    Bone *curr_bone;
    frame3f pose, rest;

    // For each bone, compute pose/rest frames
    for(int i = 0; i < skinned->bones.size(); i++) {
        curr_bone = skinned->bones[i];

        pose = curr_bone->frame_rest;
        pose.o = zero3f;

        rest = curr_bone->frame_rest;

        // Perform rotation/translation by the origin of the rest frame
        TransformedSurface *tf = new TransformedSurface();
        tf->rotation_euler = curr_bone->rotation_euler;
        tf->anim_rotation_euler = curr_bone->anim_rotation_euler;
        tf->translation = curr_bone->frame_rest.o;
        tf->scale = one3f;
        pose = transform_frame(transformed_matrix(tf, time), pose);

        // Transform wrt the pose frame of parent bone, if there is one
        if(curr_bone->parent >= 0)
            pose = transform_frame(pose_frames[curr_bone->parent], pose);
        pose_frames[i] = pose;

        // Transform wrt the rest frame of parent bone, if there is one
        if(curr_bone->parent >= 0)
            rest = transform_frame(rest_frames[curr_bone->parent], rest);
        rest_frames[i] = rest;
    }
}

/// Computes the pose position for skinned mesh
/// @param skinned The skinned mesh to update
/// @param time The time at which to compute each bone's pose frame
void skinned_update_pose(SkinnedSurface* skinned, float time) {
    if(time == skinned->_posed_cached_time) return;
    skinned->_posed_cached_time = time;

    vector<frame3f> pose_frames, rest_frames;
    skinned_bone_frames(skinned, time, pose_frames, rest_frames);

    auto& rest_pos = *shape_get_pos(skinned->shape);            // Vertex rest position
    auto& pose_pos = *shape_get_pos(skinned->_posed_cached);    // Vertex pose position

    vec3f acc; // Accumulator for positions of vertices

    // Loop through all vertices, calculating the pose for each vertex pose position
    for(int i = 0; i < rest_pos.size(); i++) {
        acc = zero3f;
        for(int j = 0; j < skinned->weights[i]->idx.size(); j++) {
            int idx = skinned->weights[i]->idx[j];
            // Transform the rest position of each vertex from object frame to rest frame of a bone
            auto pos = transform_point_inverse(rest_frames[idx], rest_pos[i]);
            // Transform from pose frame of the bone back to object frame
            pos = transform_point(pose_frames[idx], pos);
            // Compute weighted sum according to the given vertex-bone weighting to get final pose position of vertex
            auto weight = skinned->weights[i]->weight[j];
            // Add to the accumulator
            acc += weight * pos;
        }
        // Set the pose position
        pose_pos[i] = acc;
    }
}

range1f primitive_animation_interval(Primitive* prim) {
    if(is<TransformedSurface>(prim)) return transformed_animation_interval(cast<TransformedSurface>(prim));
    else if(is<InterpolatedSurface>(prim)) return range1f(0,cast<InterpolatedSurface>(prim)->shapes.size()/cast<InterpolatedSurface>(prim)->fps);
    else if(is<SkinnedSurface>(prim)) return skinned_animation_interval(cast<SkinnedSurface>(prim));
    else return range1f();
}

void primitive_simulation_init(Primitive* prim) {
    if(is<SimulatedSurface>(prim)) {
        auto simulated = cast<SimulatedSurface>(prim);
        
        // Clear the particles
        simulated->_simulator = new ParticleSimulator();
        
        // Setup forces
        simulated->_simulator->force = [simulated](const Particle& p) -> vec3f {
            vec3f force = zero3f;
            // Compute the forces acting on the particle (ex: gravity, wind)
            force = (simulated->force_wind - p.vel) * simulated->force_airfriction + simulated->force_gravity * p.mass;
            return force;
        };

        if(is<ParticleSystem>(simulated)) {
            auto psys = cast<ParticleSystem>(prim);
            psys->_simulator->begin_update = [psys](float dt){
                int n = round(dt*psys->_rng.next_int(psys->particles_per_sec));     // number of particles to create
                int i;

                // Delete particles that need to be killed off (i.e., if their timer <= 0)
                for(i = psys->_simulator->particles.size() - 1; i >= 0; i--) {
                    if(psys->_simulator->particles[i].timer <= 0)
                        psys->_simulator->particles.erase(psys->_simulator->particles.begin() + i);
                }

                // Create particles, with initial values that I came up with to
                // roughly match the reference program
                for(i = 0; i < n; i++) {
                    Particle particle;
                    auto point = shape_sample_uniform(psys->source_shape, psys->_rng.next_vec2f());
                    particle.pos = point.frame.o;
                    particle.norm = point.frame.z;
                    particle.timer = psys->_rng.next_float(psys->particles_init_timer); // so that the time-to-live of the particle roughly matches the test
                    particle.radius = psys->_rng.next_float(psys->particles_init_radius); // so that the size of the particle roughly matches the test
                    psys->_simulator->particles.push_back(particle);
                }
            };
            psys->_simulator->end_update = [psys](float dt){
                psys->_points->pos.resize(psys->_simulator->particles.size());
                psys->_points->radius.resize(psys->_simulator->particles.size());
                for(int i = 0; i < psys->_simulator->particles.size(); i ++) {
                    psys->_points->pos[i] = psys->_simulator->particles[i].pos;
                    psys->_points->radius[i] = psys->_simulator->particles[i].radius;
                }
            };
        }
        else if(is<Cloth>(simulated)) {
            auto cloth = cast<Cloth>(prim);
            
            // synch particles
            cloth->_simulator->particles.resize(cloth->_mesh->pos.size());
            for(int i = 0; i < cloth->_simulator->particles.size(); i ++) {
                cloth->_simulator->particles[i] = Particle();
                cloth->_simulator->particles[i].pos = cloth->_mesh->pos[i];
                cloth->_simulator->particles[i].norm = cloth->_mesh->norm[i];
                cloth->_simulator->particles[i].vel = zero3f;
                cloth->_simulator->particles[i].radius = (cloth->source_size.x/cloth->source_grid.x+cloth->source_size.y/cloth->source_grid.y)/2;
                cloth->_simulator->particles[i].mass = cloth->cloth_density/(cloth->_simulator->particles[i].radius*cloth->_simulator->particles[i].radius);
                cloth->_simulator->particles[i].oriented = true;
                cloth->_simulator->particles[i].pinned = false;
                cloth->_simulator->particles[i].timer = 0;
            }
            for(auto p : cloth->pinned) cloth->_simulator->particles[p].pinned = true;
            
            // setup constraints
            for(int j = 0; j < cloth->source_grid.y; j ++) {
                for(int i = 0; i < cloth->source_grid.x; i ++) {
                    int idx0 = (j+0)*(cloth->source_grid.x+1)+(i+0);
                    int idx1 = (j+0)*(cloth->source_grid.x+1)+(i+1);
                    int idx2 = (j+1)*(cloth->source_grid.x+1)+(i+1);
                    int idx3 = (j+1)*(cloth->source_grid.x+1)+(i+0);
                    cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_stretch, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx1].pos), idx0, idx1 });
                    cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_stretch, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx3].pos), idx0, idx3 });
                    cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_shear, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx2].pos), idx0, idx2 });
                    cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_shear, cloth->cloth_dump, dist(cloth->_simulator->particles[idx1].pos,cloth->_simulator->particles[idx3].pos), idx1, idx3 });
                    if(j+2 <= cloth->source_grid.y) {
                        int idx3p = (j+2)*(cloth->source_grid.x+1)+(i+0);
                        cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_bend, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx3p].pos), idx0, idx3p });
                    }
                    if(i+2 <= cloth->source_grid.x) {
                        int idx1p = (j+0)*(cloth->source_grid.x+1)+(i+2);
                        cloth->_simulator->springs.push_back(ParticleSpring { cloth->cloth_bend, cloth->cloth_dump, dist(cloth->_simulator->particles[idx0].pos,cloth->_simulator->particles[idx1p].pos), idx0, idx1p });
                    }
                }
            }
            
            cloth->_simulator->end_step = [cloth](float dt){
                for(int i = 0; i < cloth->_mesh->pos.size(); i ++) {
                    cloth->_mesh->pos[i] = cloth->_simulator->particles[i].pos;
                }
                shape_smooth_frames(cloth->_mesh);
                for(int i = 0; i < cloth->_mesh->pos.size(); i ++) {
                    cloth->_simulator->particles[i].norm = cloth->_mesh->norm[i];
                }
            };
        }
        else not_implemented_error();
    }
    else return;
}

void primitive_simulation_update(Primitive* prim, float dt) {
    if(is<SimulatedSurface>(prim)) simulator_update(cast<SimulatedSurface>(prim)->_simulator,dt);
}
