#[cfg(all(feature = "3d", feature = "default-collider"))]
use crate::math::Real;
use crate::prelude::*;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use approx::assert_relative_eq;
use bevy::{
    ecs::schedule::{LogLevel, ScheduleBuildSettings, ScheduleLabel},
    prelude::*,
    time::TimeUpdateStrategy,
};
use core::time::Duration;

#[cfg(all(feature = "2d", feature = "enhanced-determinism"))]
mod determinism_2d;

fn create_app() -> App {
    let mut app = App::new();

    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default(),
        bevy::asset::AssetPlugin::default(),
        #[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
        bevy::mesh::MeshPlugin,
        #[cfg(feature = "bevy_scene")]
        bevy::scene::ScenePlugin,
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        1.0 / 60.0,
    )));

    app.finish();

    app
}

fn tick_app(app: &mut App, timestep: f64) {
    let strategy = TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(timestep));

    if let Some(mut update_strategy) = app.world_mut().get_resource_mut::<TimeUpdateStrategy>() {
        *update_strategy = strategy;
    } else {
        app.insert_resource(strategy);
    }

    app.update();
}

#[cfg(all(feature = "3d", feature = "default-collider"))]
fn setup_cubes_simulation(mut commands: Commands) {
    let mut next_id = 0;
    // copied from "cubes" example
    let floor_size = Vec3::new(80.0, 1.0, 80.0);
    commands.spawn((
        RigidBody::Static,
        Position(RVector::NEG_Y),
        Collider::cuboid(floor_size.x, floor_size.y, floor_size.z),
    ));

    let radius = 1.0;
    let count_x = 4;
    let count_y = 4;
    let count_z = 4;
    for y in 0..count_y {
        for x in 0..count_x {
            for z in 0..count_z {
                let pos = RVector::new(
                    (x as Real - count_x as Real * 0.5) * 2.1 * radius as Real,
                    10.0 * radius as Real * y as Real,
                    (z as Real - count_z as Real * 0.5) * 2.1 * radius as Real,
                );
                commands.spawn((
                    Transform::default(),
                    RigidBody::Dynamic,
                    Position(pos + RVector::Y * 5.0),
                    Collider::cuboid(radius * 2.0, radius * 2.0, radius * 2.0),
                    Id(next_id),
                ));
                next_id += 1;
            }
        }
    }
}

#[test]
fn it_loads_plugin_without_errors() -> Result<(), Box<dyn core::error::Error>> {
    let mut app = create_app();

    for _ in 0..500 {
        tick_app(&mut app, 1.0 / 60.0);
    }

    Ok(())
}

#[test]
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
fn body_with_velocity_moves() {
    let mut app = create_app();

    app.insert_resource(Gravity::ZERO);

    app.add_systems(Startup, |mut commands: Commands| {
        // move right at 1 unit per second
        commands.spawn((
            Transform::default(),
            RigidBody::Dynamic,
            LinearVelocity(Vector::X),
            #[cfg(feature = "2d")]
            MassPropertiesBundle::from_shape(&Circle::new(0.5), 1.0),
            #[cfg(feature = "3d")]
            MassPropertiesBundle::from_shape(&Sphere::new(0.5), 1.0),
        ));
    });

    // Run startup systems
    app.update();

    const UPDATES: usize = 500;

    for _ in 0..UPDATES {
        tick_app(&mut app, 1.0 / 60.0);
    }

    let mut app_query = app.world_mut().query::<(&Transform, &RigidBody)>();

    let (transform, _body) = app_query.single(app.world()).unwrap();

    assert_relative_eq!(transform.translation.y, 0.);
    assert_relative_eq!(transform.translation.z, 0.);

    // make sure we end up in the expected position
    assert_relative_eq!(
        transform.translation.x,
        1. * UPDATES as f32 * 1. / 60.,
        epsilon = 0.03 // allow some leeway, as we might be one frame off
    );
}

#[derive(Component, Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
#[cfg(all(feature = "3d", feature = "default-collider"))]
struct Id(usize);

#[cfg(all(feature = "3d", feature = "default-collider"))]
#[test]
fn cubes_simulation_is_locally_deterministic() {
    use itertools::Itertools;

    fn run_cubes() -> Vec<(Id, Transform)> {
        let mut app = create_app();

        app.add_systems(Startup, setup_cubes_simulation);

        // Run startup systems
        app.update();

        const SECONDS: usize = 5;
        const UPDATES: usize = 60 * SECONDS;

        for _ in 0..UPDATES {
            tick_app(&mut app, 1.0 / 60.0);
        }

        let mut app_query = app.world_mut().query::<(&Id, &Transform)>();

        let mut bodies: Vec<(Id, Transform)> = app_query
            .iter(app.world())
            .map(|(id, transform)| (*id, *transform))
            .collect();
        bodies.sort_by_key(|b| b.0);
        bodies
    }

    // run simulation and check that results are equal each time
    for (a, b) in (0..4).map(|_| run_cubes()).tuple_windows() {
        assert_eq!(a, b);
    }
}

/// A box resting on a 14° slope, with friction twice the tan 14° ≈ 0.25 it needs to hold and kept
/// awake, must stay where it settled.
///
/// The bound comes from the solver's precision: its velocities are `f32`. Each step, gravity adds
/// g sin 14° Δt ≈ 0.04 m/s along the slope, which friction takes away again; rounding leaves a
/// residual of a few ulps of that, under 1e-7 m/s even across the substeps. If every one of the
/// 540 measured steps left that residual in the same direction, the box would move
/// 540 × 1e-7 m/s × (1/60) s ≈ 0.9 µm. The bound is 1 µm. With `f64` positions that is all.
/// With `f32` positions (the default features), a step that moves the box by less than half an
/// ulp of its position, 3e-8 m near 1 m, leaves it where it was; the residual moves it at most
/// 1.7e-9 m a step, so rounding can drop that creep but not add to it, and the bound holds for
/// both. A friction warm start applied in a different tangent basis than it was solved in makes
/// the box creep by tens of millimetres here.
#[cfg(all(feature = "3d", feature = "default-collider"))]
#[test]
fn box_held_by_friction_on_slope_does_not_creep() {
    use crate::physics_transform::PhysicsTransformConfig;

    const HZ: f64 = 60.0;

    let mut app = create_app();
    app.insert_resource(PhysicsTransformConfig {
        transform_to_position: false,
        ..default()
    })
    .insert_resource(Time::<Fixed>::from_hz(HZ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        1.0 / HZ,
    )));

    let tilt = Quat::from_rotation_z(14f32.to_radians());
    app.world_mut().spawn((
        RigidBody::Static,
        Collider::cuboid(40.0, 1.0, 40.0),
        Friction::new(0.5),
        Rotation(tilt),
    ));
    let body = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::cuboid(1.0, 1.0, 1.0),
            Friction::new(0.5),
            Position(RVector::from(tilt * Vec3::Y)),
            Rotation(tilt),
            SleepingDisabled,
        ))
        .id();

    let position = |app: &App| app.world().get::<Position>(body).unwrap().0;

    // Let the box settle onto the slope for a second.
    for _ in 0..HZ as usize {
        app.update();
    }
    let settled = position(&app);

    for _ in 0..9 * HZ as usize {
        app.update();
    }

    let normal = RVector::from(tilt * Vec3::Y);
    let moved = position(&app) - settled;
    let creep = (moved - normal * moved.dot(normal)).length();
    assert!(
        creep < 1e-6,
        "the box crept {:.6} mm along the slope in 9 s",
        creep * 1e3
    );
}

/// A box that falls asleep on a slab, and is woken with the slab taken away, falls freely from rest
/// in its first awake step: its velocity changes by one step's gravity, g Δt, and no more.
///
/// The bound comes from the solver's precision: velocities and gravity are `f32` in both
/// precisions. The step adds g h to the velocity at each of its 6 substeps of h = Δt / 6; the
/// increment rounds to within 2 ulps of g h, and each of the 6 sums to within half an ulp of
/// g Δt ≈ 0.16 m/s (7.5e-9 m/s), which leaves under 1e-7 m/s in all. The bound is 1e-6 m/s.
/// Velocity increments gathered while asleep and kept for the step the body wakes in add
/// g h² / (1 - h) per substep, 4.6e-4 m/s over this step.
#[cfg(feature = "default-collider")]
#[test]
fn woken_body_takes_one_steps_gravity() {
    const HZ: f64 = 60.0;

    let mut app = create_app();
    app.insert_resource(Time::<Fixed>::from_hz(HZ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / HZ,
        )));

    let slab = app
        .world_mut()
        .spawn((
            RigidBody::Static,
            #[cfg(feature = "2d")]
            Collider::rectangle(10.0, 1.0),
            #[cfg(feature = "3d")]
            Collider::cuboid(10.0, 1.0, 10.0),
        ))
        .id();
    let body = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            #[cfg(feature = "2d")]
            Collider::rectangle(1.0, 1.0),
            #[cfg(feature = "3d")]
            Collider::cuboid(1.0, 1.0, 1.0),
            Position(RVector::Y),
        ))
        .id();

    // Let the box settle and fall asleep, then sleep for another second.
    for _ in 0..5 * HZ as usize {
        app.update();
        if app.world().get::<Sleeping>(body).is_some() {
            break;
        }
    }
    assert!(
        app.world().get::<Sleeping>(body).is_some(),
        "the box did not fall asleep"
    );
    for _ in 0..HZ as usize {
        app.update();
    }
    assert!(app.world().get::<Sleeping>(body).is_some());

    // Take the slab away and wake the box.
    app.world_mut().despawn(slab);
    app.world_mut().entity_mut(body).remove::<Sleeping>();

    let velocity = |app: &App| app.world().get::<LinearVelocity>(body).unwrap().0;
    let before = velocity(&app);
    app.update();
    let change = velocity(&app) - before;

    let gravity = Vector::NEG_Y * 9.81 * (1.0 / HZ) as f32;
    assert!(
        (change - gravity).length() < 1e-6,
        "the first awake step changed the velocity by {change:?}, one step's gravity is {gravity:?}"
    );
}

/// A ball dropped onto the ground at 3 m/s, with restitution 0.5 and no friction, rebounds once at
/// 1.5 m/s and then rebounds no faster than half of what it lands with.
///
/// After the first rebound only gravity and the contact act on the ball. Gravity gives back in
/// the fall what the rise took, and the contact adds no energy but restitution's, so the ball
/// lands again at most as fast as it left, 1.5 m/s, and leaves that landing at most at
/// e × 1.5 = 0.75 m/s. It rises 1.1 cm, within the narrow phase's 5 cm recycling distance, so
/// every later step recycles its contact: a recycled point that kept the normal speed of the
/// 3 m/s landing makes restitution repeat that landing's 1.5 m/s rebound at every step.
#[cfg(feature = "default-collider")]
#[test]
fn recycled_contact_does_not_repeat_an_old_bounce() {
    const HZ: f64 = 60.0;

    let mut app = create_app();
    app.insert_resource(Time::<Fixed>::from_hz(HZ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / HZ,
        )))
        .insert_resource(Gravity(Vector::NEG_Y * 100.0));

    app.world_mut().spawn((
        RigidBody::Static,
        #[cfg(feature = "2d")]
        Collider::rectangle(20.0, 1.0),
        #[cfg(feature = "3d")]
        Collider::cuboid(20.0, 1.0, 20.0),
        Position(RVector::NEG_Y * 0.5),
        Restitution::new(0.5),
        Friction::new(0.0),
    ));
    let ball = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            #[cfg(feature = "2d")]
            Collider::circle(0.5),
            #[cfg(feature = "3d")]
            Collider::sphere(0.5),
            Position(RVector::Y * 0.51),
            LinearVelocity(Vector::NEG_Y * 3.0),
            Restitution::new(0.5),
            Friction::new(0.0),
        ))
        .id();

    let rise = |app: &App| app.world().get::<LinearVelocity>(ball).unwrap().y;

    // The first landing: restitution sends it off at e × 3 m/s, which the solver sets to within
    // rounding; 1 cm/s is only a check that this is the landing the bound below starts from.
    let mut steps = 0;
    while rise(&app) <= 0.0 {
        app.update();
        steps += 1;
        assert!(steps < 10, "the ball did not rebound");
    }
    assert!(
        (rise(&app) - 1.5).abs() < 0.01,
        "it rebounded at {}",
        rise(&app)
    );

    for step in 0..40 {
        app.update();
        assert!(
            rise(&app) < 0.75,
            "{step} steps after the first rebound, the ball rose at {} m/s",
            rise(&app)
        );
    }
}

/// A plank resting flat on the ground and set turning about its centre reads, at each point of
/// its recycled contact, the speed at which the turn moves that point off the ground or into it.
///
/// The plank lies still, so the narrow phase keeps its contact from the step before, with the
/// normal speed of a plank at rest. Turning at ω about the axis across its length, it lifts the
/// point x along it from its centre at ω x and drives the point at -x down as fast: a recycled
/// point that kept its old normal speed reads about 0, and one that mixed up the turn's direction
/// reads -ω x. The plank's velocity is set exactly, so the bound is only the rounding of the
/// cross product, far under the 0.5 m/s the turn gives the ends.
#[cfg(feature = "default-collider")]
#[test]
fn recycled_contact_reads_the_bodies_current_turn() {
    use crate::collision::CollisionDiagnostics;
    use crate::math::ToF32Precision;

    const HZ: f64 = 60.0;
    const OMEGA: f32 = 0.5;

    #[derive(Resource, Clone, Copy)]
    struct Pair(Entity, Entity);

    /// Each point's distance along the plank from its centre and normal speed, and whether the
    /// narrow phase recycled the contact, as the solver gets them.
    #[derive(Resource, Default)]
    struct Read(Vec<(f32, f32)>, bool);

    // One physics step per update, so that the step read is the one the turn was set for.
    let mut app = create_app();
    app.insert_resource(Time::<Fixed>::from_hz(HZ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / HZ,
        )));
    app.init_resource::<Read>().add_systems(
        PhysicsSchedule,
        (|world: &mut World| {
            let Pair(ground, plank) = *world.resource::<Pair>();
            let points = world.resource::<ContactGraph>().get(ground, plank).map_or(
                Vec::new(),
                |(_, pair)| {
                    pair.manifolds
                        .iter()
                        .flat_map(|manifold| manifold.points.iter())
                        .map(|point| (point.point.x.f32(), point.normal_speed))
                        .collect()
                },
            );
            let recycled = world.resource::<CollisionDiagnostics>().recycled_count > 0;
            *world.resource_mut::<Read>() = Read(points, recycled);
        })
        .after(PhysicsStepSystems::NarrowPhase)
        .before(PhysicsStepSystems::Solver),
    );

    let ground = app
        .world_mut()
        .spawn((
            RigidBody::Static,
            #[cfg(feature = "2d")]
            Collider::rectangle(20.0, 1.0),
            #[cfg(feature = "3d")]
            Collider::cuboid(20.0, 1.0, 20.0),
            Position(RVector::NEG_Y * 0.5),
        ))
        .id();
    let plank = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            #[cfg(feature = "2d")]
            Collider::rectangle(2.0, 0.2),
            #[cfg(feature = "3d")]
            Collider::cuboid(2.0, 0.2, 0.4),
            Position(RVector::Y * 0.1),
            SleepingDisabled,
        ))
        .id();
    app.insert_resource(Pair(ground, plank));

    for _ in 0..30 {
        app.update();
    }
    let mut body = app.world_mut().entity_mut(plank);
    body.insert(LinearVelocity::ZERO);
    #[cfg(feature = "2d")]
    body.insert(AngularVelocity(OMEGA));
    #[cfg(feature = "3d")]
    body.insert(AngularVelocity(Vector::Z * OMEGA));
    let centre = body.get::<Position>().unwrap().x.f32();
    app.update();

    let Read(points, recycled) = app.world().resource::<Read>();
    assert!(*recycled, "the resting plank's contact was not recycled");
    assert!(!points.is_empty(), "the plank was not on the ground");
    for &(x, normal_speed) in points {
        let along = x - centre;
        assert!(
            (normal_speed - OMEGA * along).abs() < 1e-3,
            "the point {along} m along the plank read {normal_speed} m/s, the turn moves it at {}",
            OMEGA * along
        );
    }
}

#[test]
fn no_ambiguity_errors() {
    #[derive(ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
    struct DeterministicSchedule;

    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        PhysicsPlugins::new(DeterministicSchedule)
            .build()
            .disable::<ColliderHierarchyPlugin>(),
        bevy::asset::AssetPlugin::default(),
        #[cfg(feature = "bevy_scene")]
        bevy::scene::ScenePlugin,
        #[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
        bevy::mesh::MeshPlugin,
    ))
    .edit_schedule(DeterministicSchedule, |s| {
        s.set_build_settings(ScheduleBuildSettings {
            ambiguity_detection: LogLevel::Error,
            ..default()
        });
    })
    .add_systems(Update, |w: &mut World| {
        w.run_schedule(DeterministicSchedule);
    })
    .finish();
    app.update();
}
