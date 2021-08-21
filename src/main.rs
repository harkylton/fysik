extern crate generational_arena;
extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate piston;

use glutin_window::GlutinWindow as Window;
use graphics::color::{NAVY, RED, SILVER};
use graphics::context::Context;
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop::{EventSettings, Events};
use piston::input::{
    Button, Key, MouseButton, MouseCursorEvent, PressEvent, ReleaseEvent, RenderArgs, RenderEvent,
    UpdateArgs, UpdateEvent,
};
use piston::window::WindowSettings;

mod body;
mod collisions;
mod material;
mod shape;
mod utils;
mod vector;
mod world;

use crate::body::{Body, BodyHandle};
use crate::material::{BOUNCY_BALL, STATIC, WOOD};
use crate::shape::Shape;
use crate::utils::radians;
use crate::vector::Vec2;
use crate::world::World;

struct DragState {
    handle: BodyHandle,
    start: Vec2,
    end: Vec2,
}

pub struct App {
    gl: GlGraphics, // OpenGL drawing backend.
    world: World,
    paused: bool,
    drag_state: Option<DragState>,
}

impl App {
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        let bodies = self.world.bodies.iter();
        let drag_state = &self.drag_state;

        self.gl.draw(args.viewport(), |c, gl| {
            clear(SILVER, gl);

            for (_, body) in bodies {
                draw_body(gl, c, &body);
            }

            if let Some(DragState { start, end, .. }) = drag_state {
                line_from_to(
                    RED,
                    1.0,
                    [start.x, start.y],
                    [end.x, end.y],
                    c.transform,
                    gl,
                );
            }
        });
    }

    fn update(&mut self, args: &UpdateArgs) {
        if !self.paused {
            self.world.update(args.dt);
        }
    }

    fn end_drag(&mut self) {
        if let Some(DragState { handle, start, end }) = self.drag_state {
            self.world.drag_body(handle, start, end);
            self.drag_state = None;
        }
    }

    fn update_drag(&mut self, position: Vec2) {
        if let Some(drag_state) = &self.drag_state {
            self.drag_state = Some(DragState {
                end: position,
                ..*drag_state
            });
        }
    }

    fn start_drag(&mut self, handle: BodyHandle, position: Vec2) {
        self.drag_state = Some(DragState {
            handle,
            start: position,
            end: position,
        })
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
        paused: false,
        drag_state: None,
    };

    app.world.add_body(Body::new(
        Shape::rect(600.0, 25.0),
        Vec2 { x: 350.0, y: 750.0 },
        STATIC,
    ));

    app.world.add_body(Body::new(
        Shape::circle(50.0),
        Vec2 { x: 350.0, y: 500.0 },
        STATIC,
    ));

    app.world.add_body(Body::new(
        Shape::Circle { r: 10.0 },
        Vec2 { x: 100.0, y: 100.0 },
        BOUNCY_BALL,
    ));
    app.world.add_body(Body::new(
        Shape::Circle { r: 20.0 },
        Vec2 { x: 101.0, y: 50.0 },
        BOUNCY_BALL,
    ));

    let mut cursor = [0.0, 0.0];
    let mut events = Events::new(EventSettings::new());

    while let Some(e) = events.next(&mut window) {
        e.mouse_cursor(|pos| {
            cursor = pos;
            app.update_drag(Vec2::new(cursor[0], cursor[1]));
        });

        if let Some(Button::Mouse(button)) = e.press_args() {
            if let MouseButton::Left = button {
                let position = Vec2::new(cursor[0], cursor[1]);
                if let Some(handle) = app.world.body_at_point(position) {
                    app.start_drag(handle, position);
                    println!("Dragstart: {:?}", cursor);
                } else {
                    app.world
                        .add_body(Body::new(Shape::Circle { r: 20.0 }, position, BOUNCY_BALL));
                }
            }

            if let MouseButton::Right = button {
                app.world.add_body(Body::new(
                    Shape::random_polygon(),
                    Vec2::new(cursor[0], cursor[1]),
                    WOOD,
                ));
            }
        }

        if let Some(Button::Mouse(button)) = e.release_args() {
            if button == MouseButton::Left {
                println!("Release: {:?}", cursor);
                app.end_drag();
            }
        };

        if let Some(Button::Keyboard(key)) = e.press_args() {
            if key == Key::P {
                println!("Toggled pause..");
                app.paused = !app.paused;
                // capture_cursor = !capture_cursor;
                // window.set_capture_cursor(capture_cursor);
            } else if key == Key::S {
                if app.paused {
                    app.world.step(1.0 / 60.0);
                }
            }
        };

        if let Some(args) = e.render_args() {
            app.render(&args);
        }

        if let Some(args) = e.update_args() {
            app.update(&args);
        }
    }
}

fn draw_body(gl: &mut GlGraphics, c: Context, body: &Body) {
    use graphics::*;

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
        }
        Shape::Polygon { vertices, .. } => {
            let poly = vertices
                .iter()
                .map(|v| v.as_slice())
                .collect::<Vec<[f64; 2]>>();
            let transform = c
                .transform
                .trans(body.position.x, body.position.y)
                .rot_rad(body.rotation);

            polygon(NAVY, &poly, transform, gl);
        }
    }
}
