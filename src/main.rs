
mod body;
mod collisions;
mod vector;
mod math;
mod material;
mod world;

use crate::body::{Body, Shape};
use crate::vector::Vec2;
use crate::material::{ROCK, WOOD, BOUNCY_BALL, STATIC};
use crate::math::radians;
use crate::world::World;

extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate piston;

use glutin_window::GlutinWindow as Window;
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop::{EventSettings, Events};
use piston::input::{RenderArgs, RenderEvent, UpdateArgs, UpdateEvent, Button, PressEvent, MouseCursorEvent, MouseButton};
use piston::window::WindowSettings;
use graphics::context::Context;
use graphics::color::{NAVY, SILVER};

pub struct App {
    gl: GlGraphics, // OpenGL drawing backend.
    world: World,
}

fn draw_body(gl: &mut GlGraphics, c: Context, body: &Body) {
    use graphics::*;
    const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];

    match &body.shape {
        Shape::Circle { r } => {
            let rect = rectangle::rectangle_by_corners(-*r, -*r, *r, *r);
            
            let transform = c
                .transform
                .trans(body.position.x, body.position.y)
                .rot_rad(body.rotation);

            circle_arc(NAVY, 1.0, 0.0, radians(360.0), rect, transform, gl);
            
            // Line to show rotation
            let a = [0.0, 0.0];
            let b = [0.0, *r];

            line_from_to(RED, 1.0, a, b, transform, gl);

        },
        Shape::Polygon { vertices, .. } => {
            let poly = vertices.iter().map(|v| { v.as_slice() }).collect::<Vec<[f64; 2]>>();
            let transform = c
                .transform
                .trans(body.position.x, body.position.y)
                .rot_rad(body.rotation);

            polygon(NAVY, &poly, transform, gl);
        }
    }
}

impl App {
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        let bodies = self.world.bodies.iter();

        self.gl.draw(args.viewport(), |c, gl| {
            clear(SILVER, gl);

            for (_, body) in bodies {
                draw_body(gl, c, &body);
            }
            
        });
    }

    fn update(&mut self, args: &UpdateArgs) {
        self.world.update();
    }
}

fn main() {
    // Change this to OpenGL::V2_1 if not working.
    let opengl = OpenGL::V3_2;

    // Create an Glutin window.
    let mut window: Window = WindowSettings::new("fysik", [1024, 800])
        .graphics_api(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    // Create a new game and run it.
    let mut app = App {
        gl: GlGraphics::new(opengl),
        world: World::new(),
    };

    app.world.add_body(
        Body::new(Shape::rect(600.0, 25.0), Vec2 { x: 350.0, y: 750.0 }, STATIC)
    );

    app.world.add_body(
        Body::new(Shape::circle(50.0), Vec2 { x: 350.0, y: 500.0 }, STATIC)
    );
    
    app.world.add_body( 
        Body::new(Shape::Circle { r: 10.0 }, Vec2 { x: 100.0, y: 100.0 }, BOUNCY_BALL),
    );
    app.world.add_body( 
        Body::new(Shape::Circle { r: 20.0 }, Vec2 { x: 101.0, y: 50.0 }, BOUNCY_BALL),
    );

    let mut cursor = [0.0, 0.0];
    let mut events = Events::new(EventSettings::new());
    while let Some(e) = events.next(&mut window) {
        e.mouse_cursor(|pos| {
            cursor = pos;
        });

        if let Some(Button::Mouse(button)) = e.press_args() {
            if let MouseButton::Left = button {
                app.world.add_body(
                    Body::new(
                        Shape::Circle { r: 20.0 }, 
                        Vec2::new(cursor[0], cursor[1]), 
                        BOUNCY_BALL,
                    ),
                );
            }

            if let MouseButton::Right = button {
                app.world.add_body(
                    Body::new(
                        Shape::random_polygon(), 
                        Vec2::new(cursor[0], cursor[1]), 
                        WOOD,
                    ),
                );
            }
        }

        if let Some(args) = e.render_args() {
            app.render(&args);
        }

        if let Some(args) = e.update_args() {
            app.update(&args);
        }
    }
}

//fn main() {
//    println!("Hello, world!");
//    
//    let bodies: Vec<Body>= vec![
//        Body::new(Shape::Rect { w: 10.0, h: 10.0 }, Vec2 { x: 0.0, y: 0.0 }, ROCK),
//        Body::new(Shape::Circle { r: 5.0 }, Vec2 { x: 20.0, y: 0.0 }, BOUNCY_BALL),
//    ];
//
//
//}
