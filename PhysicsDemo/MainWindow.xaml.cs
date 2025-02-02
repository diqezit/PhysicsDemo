using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows.Input;
using System.Windows.Controls;

namespace PhysicsDemo
{
    /// <summary>
    /// Main window class for the physics simulation demonstration.
    /// Implements a 2D physics simulation with collision detection and response.
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Constants and Structures

        /// <summary>
        /// Contains physical constants used throughout the simulation.
        /// </summary>
        private readonly struct PhysicsConstants
        {
            public const double CircleRadius = 10;
            public const double Restitution = 0.8;
            public const double TangentFriction = 0.9;
            public const double RotationSpeed = 30;
            public static readonly Vector Gravity = new Vector(0, 500);
        }

        /// <summary>
        /// Encapsulates the current state of the physics simulation.
        /// </summary>
        private class SimulationState
        {
            public List<Point> ContainerPolygon { get; set; }
            public double ContainerAngle { get; set; }
            public Vector CirclePosition { get; set; }
            public Vector CircleVelocity { get; set; }
            public Point ContainerCenter { get; set; }

            public SimulationState()
            {
                ContainerPolygon = new List<Point>();
            }
        }

        #endregion

        #region Private Fields

        private readonly SimulationState _state = new SimulationState();
        private readonly Stopwatch _stopwatch = new Stopwatch();
        private TimeSpan _lastTime;
        private string _selectedShape = "Square";

        // Drag state
        private bool _isDragging;
        private Point _dragStart;
        private Point _containerCenterInitial;

        // UI elements
        private Path _containerPath;
        private Ellipse _circleEllipse;

        #endregion

        #region Initialization

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
        }

        /// <summary>
        /// Handles the window loaded event.
        /// </summary>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            InitializeSimulation();
            InitializeVisuals();
            StartSimulation();
        }

        /// <summary>
        /// Initializes the simulation state.
        /// </summary>
        private void InitializeSimulation()
        {
            InitShape();
            ResetCircle();
        }

        /// <summary>
        /// Initializes visual elements of the simulation.
        /// </summary>
        private void InitializeVisuals()
        {
            _containerPath = CreateContainerPath();
            _circleEllipse = CreateCircleEllipse();
            SimulationCanvas.Children.Add(_containerPath);
            SimulationCanvas.Children.Add(_circleEllipse);
        }

        /// <summary>
        /// Creates and configures the container path visual element.
        /// </summary>
        private Path CreateContainerPath() => new Path
        {
            Stroke = Brushes.DarkSlateGray,
            StrokeThickness = 3,
            SnapsToDevicePixels = true,
            StrokeLineJoin = PenLineJoin.Round
        };

        /// <summary>
        /// Creates and configures the circle visual element.
        /// </summary>
        private Ellipse CreateCircleEllipse() => new Ellipse
        {
            Width = PhysicsConstants.CircleRadius * 2,
            Height = PhysicsConstants.CircleRadius * 2,
            Fill = new RadialGradientBrush(Colors.IndianRed, Colors.DarkRed)
            {
                GradientOrigin = new Point(0.3, 0.3)
            },
            Effect = new System.Windows.Media.Effects.DropShadowEffect
            {
                Color = Colors.Black,
                BlurRadius = 5,
                ShadowDepth = 2
            }
        };

        /// <summary>
        /// Starts the physics simulation.
        /// </summary>
        private void StartSimulation()
        {
            _stopwatch.Start();
            _lastTime = _stopwatch.Elapsed;
            CompositionTarget.Rendering += OnRendering;
        }

        #endregion

        #region Shape Management

        /// <summary>
        /// Handles the shape selection change event.
        /// </summary>
        private void ShapeComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (ShapeComboBox.SelectedItem is ComboBoxItem item)
            {
                _selectedShape = item.Content.ToString();
                InitializeSimulation();
            }
        }

        /// <summary>
        /// Resets the circle position and velocity to initial values.
        /// </summary>
        private void ResetCircle()
        {
            if (!IsCanvasReady())
            {
                Dispatcher.BeginInvoke(new Action(ResetCircle), System.Windows.Threading.DispatcherPriority.Loaded);
                return;
            }

            double cx = SimulationCanvas.ActualWidth / 2;
            double cy = SimulationCanvas.ActualHeight / 2;
            _state.ContainerCenter = new Point(cx, cy);
            _state.CirclePosition = new Vector(cx, cy - 50);
            _state.CircleVelocity = new Vector(0, 0);
        }

        /// <summary>
        /// Checks if the canvas is ready for rendering.
        /// </summary>
        private bool IsCanvasReady() =>
            SimulationCanvas != null && SimulationCanvas.ActualWidth > 0 && SimulationCanvas.ActualHeight > 0;

        /// <summary>
        /// Initializes the shape based on the selected type.
        /// </summary>
        private void InitShape()
        {
            _state.ContainerPolygon.Clear();
            switch (_selectedShape)
            {
                case "Square":
                    CreateSquare();
                    break;
                case "Triangle":
                    CreateTriangle();
                    break;
                case "Pentagon":
                    CreatePentagon();
                    break;
                default:
                    CreateSquare();
                    break;
            }
        }

        /// <summary>
        /// Creates a square shape.
        /// </summary>
        private void CreateSquare()
        {
            _state.ContainerPolygon.AddRange(new[]
            {
                new Point(-100, -100),
                new Point(100, -100),
                new Point(100, 100),
                new Point(-100, 100)
            });
        }

        /// <summary>
        /// Creates a triangle shape.
        /// </summary>
        private void CreateTriangle()
        {
            _state.ContainerPolygon.AddRange(new[]
            {
                new Point(0, -120),
                new Point(120, 100),
                new Point(-120, 100)
            });
        }

        /// <summary>
        /// Creates a pentagon shape.
        /// </summary>
        private void CreatePentagon()
        {
            const int sides = 5;
            const double radius = 120;
            for (int i = 0; i < sides; i++)
            {
                double angle = i * 2 * Math.PI / sides - Math.PI / 2;
                _state.ContainerPolygon.Add(new Point(
                    radius * Math.Cos(angle),
                    radius * Math.Sin(angle)));
            }
        }

        #endregion

        #region Physics Simulation

        /// <summary>
        /// Handles the rendering event for physics updates.
        /// </summary>
        private void OnRendering(object sender, EventArgs e)
        {
            double dt = CalculateDeltaTime();
            UpdatePhysics(dt);
            UpdateVisuals();
        }

        /// <summary>
        /// Calculates the time delta between frames.
        /// </summary>
        private double CalculateDeltaTime()
        {
            TimeSpan currentTime = _stopwatch.Elapsed;
            double dt = (currentTime - _lastTime).TotalSeconds;
            _lastTime = currentTime;
            return dt;
        }

        /// <summary>
        /// Updates the physics simulation state.
        /// </summary>
        private void UpdatePhysics(double dt)
        {
            _state.ContainerAngle = (_state.ContainerAngle + PhysicsConstants.RotationSpeed * dt) % 360;
            _state.CircleVelocity += PhysicsConstants.Gravity * dt;
            _state.CirclePosition += _state.CircleVelocity * dt;

            var containerTransform = CreateContainerTransform();
            (Point localPos, Vector localVel) = TransformToLocal(_state.CirclePosition, _state.CircleVelocity, containerTransform);

            if (ProcessCollisions(ref localPos, ref localVel))
            {
                (Vector worldVel, Point worldPos) = TransformToWorld(localVel, localPos, containerTransform);
                _state.CircleVelocity = worldVel;
                _state.CirclePosition = new Vector(worldPos.X, worldPos.Y);
            }
        }

        #endregion

        #region Transformation Helpers

        /// <summary>
        /// Creates the container transform group.
        /// </summary>
        private TransformGroup CreateContainerTransform()
        {
            var transform = new TransformGroup();
            transform.Children.Add(new RotateTransform(_state.ContainerAngle));
            transform.Children.Add(new TranslateTransform(_state.ContainerCenter.X, _state.ContainerCenter.Y));
            return transform;
        }

        /// <summary>
        /// Transforms world coordinates to local space.
        /// </summary>
        private (Point localPos, Vector localVel) TransformToLocal(Vector worldPos, Vector worldVel, TransformGroup containerTransform)
        {
            Matrix invMatrix = containerTransform.Value;
            invMatrix.Invert();
            Point localPos = invMatrix.Transform(new Point(worldPos.X, worldPos.Y));
            Matrix invRotate = new RotateTransform(-_state.ContainerAngle).Value;
            Vector localVel = invRotate.Transform(worldVel);
            return (localPos, localVel);
        }

        /// <summary>
        /// Transforms local coordinates to world space.
        /// </summary>
        private (Vector worldVel, Point worldPos) TransformToWorld(Vector localVel, Point localPos, TransformGroup containerTransform)
        {
            Matrix rotateMatrix = new RotateTransform(_state.ContainerAngle).Value;
            Vector worldVel = rotateMatrix.Transform(localVel);
            Point worldPos = containerTransform.Value.Transform(localPos);
            return (worldVel, worldPos);
        }

        #endregion

        #region Collision Detection and Response

        /// <summary>
        /// Processes collisions between the circle and container edges.
        /// </summary>
        private bool ProcessCollisions(ref Point localPos, ref Vector localVel)
        {
            bool collided = false;
            foreach (var edge in GetPolygonEdges(_state.ContainerPolygon))
            {
                if (ProcessEdgeCollision(edge, ref localPos, ref localVel))
                    collided = true;
            }
            return collided;
        }

        /// <summary>
        /// Processes collision with a single edge.
        /// </summary>
        private bool ProcessEdgeCollision(Tuple<Point, Point> edge, ref Point localPos, ref Vector localVel)
        {
            Vector edgeVec = edge.Item2 - edge.Item1;
            double edgeLength = edgeVec.Length;
            if (edgeLength == 0) return false;

            Vector edgeDir = edgeVec / edgeLength;
            Vector toCircle = localPos - edge.Item1;
            double proj = Vector.Multiply(toCircle, edgeDir);
            proj = Math.Max(0, Math.Min(edgeLength, proj));
            Point closest = edge.Item1 + edgeDir * proj;
            Vector diff = localPos - closest;
            double distance = diff.Length;

            if (distance >= PhysicsConstants.CircleRadius) return false;

            Vector normal = diff.Length > 0 ? diff / diff.Length : new Vector(0, -1);
            double vn = Vector.Multiply(localVel, normal);
            Vector vNormal = vn * normal;
            Vector vTangent = localVel - vNormal;

            double newVn = -PhysicsConstants.Restitution * vn;
            localVel = newVn * normal + vTangent * PhysicsConstants.TangentFriction;
            localPos += normal * (PhysicsConstants.CircleRadius - distance);
            return true;
        }

        #endregion

        #region Visual Updates

        /// <summary>
        /// Updates all visual elements.
        /// </summary>
        private void UpdateVisuals()
        {
            var containerTransform = CreateContainerTransform();
            UpdateContainerVisual(containerTransform);
            UpdateCirclePosition();
        }

        /// <summary>
        /// Updates the container visual representation.
        /// </summary>
        private void UpdateContainerVisual(Transform transform)
        {
            PathGeometry geometry = new PathGeometry();
            if (_state.ContainerPolygon.Count > 0)
            {
                PathFigure figure = new PathFigure
                {
                    StartPoint = transform.Transform(_state.ContainerPolygon[0]),
                    IsClosed = true,
                    IsFilled = false
                };

                for (int i = 1; i < _state.ContainerPolygon.Count; i++)
                {
                    figure.Segments.Add(new LineSegment(transform.Transform(_state.ContainerPolygon[i]), true));
                }
                geometry.Figures.Add(figure);
            }
            _containerPath.Data = geometry;
        }

        /// <summary>
        /// Updates the circle position on the canvas.
        /// </summary>
        private void UpdateCirclePosition()
        {
            Canvas.SetLeft(_circleEllipse, _state.CirclePosition.X - PhysicsConstants.CircleRadius);
            Canvas.SetTop(_circleEllipse, _state.CirclePosition.Y - PhysicsConstants.CircleRadius);
        }

        /// <summary>
        /// Gets the edges of a polygon as point pairs.
        /// </summary>
        private IEnumerable<Tuple<Point, Point>> GetPolygonEdges(List<Point> polygon)
        {
            for (int i = 0; i < polygon.Count; i++)
            {
                yield return new Tuple<Point, Point>(
                    polygon[i],
                    polygon[(i + 1) % polygon.Count]);
            }
        }

        #endregion

        #region Mouse Event Handlers

        /// <summary>
        /// Handles mouse button down event for container dragging.
        /// </summary>
        private void SimulationCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point mousePos = e.GetPosition(SimulationCanvas);
            var worldPolygon = GetWorldPolygon();

            if (IsPointInPolygon(mousePos, worldPolygon))
            {
                StartDragging(mousePos);
            }
        }

        /// <summary>
        /// Gets the container polygon vertices in world coordinates.
        /// </summary>
        /// <returns>List of polygon vertices in world coordinates.</returns>
        private List<Point> GetWorldPolygon()
        {
            var containerTransform = CreateContainerTransform();
            return _state.ContainerPolygon.ConvertAll(p => containerTransform.Transform(p));
        }

        /// <summary>
        /// Initiates the dragging operation.
        /// </summary>
        /// <param name="mousePos">The current mouse position.</param>
        private void StartDragging(Point mousePos)
        {
            _isDragging = true;
            _dragStart = mousePos;
            _containerCenterInitial = _state.ContainerCenter;
            SimulationCanvas.CaptureMouse();
        }

        /// <summary>
        /// Handles mouse movement for container dragging.
        /// </summary>
        private void SimulationCanvas_MouseMove(object sender, MouseEventArgs e)
        {
            if (_isDragging)
            {
                Point currentPos = e.GetPosition(SimulationCanvas);
                Vector delta = currentPos - _dragStart;
                _state.ContainerCenter = new Point(
                    _containerCenterInitial.X + delta.X,
                    _containerCenterInitial.Y + delta.Y);
            }
        }

        /// <summary>
        /// Handles mouse button up event to end container dragging.
        /// </summary>
        private void SimulationCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (_isDragging)
            {
                _isDragging = false;
                SimulationCanvas.ReleaseMouseCapture();
            }
        }

        /// <summary>
        /// Determines if a point lies within a polygon using the ray-casting algorithm.
        /// </summary>
        /// <param name="pt">The point to test.</param>
        /// <param name="polygon">The polygon vertices.</param>
        /// <returns>True if the point is inside the polygon, false otherwise.</returns>
        private bool IsPointInPolygon(Point pt, List<Point> polygon)
        {
            bool inside = false;
            for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            {
                if (((polygon[i].Y > pt.Y) != (polygon[j].Y > pt.Y)) &&
                    (pt.X < (polygon[j].X - polygon[i].X) * (pt.Y - polygon[i].Y) /
                    (polygon[j].Y - polygon[i].Y) + polygon[i].X))
                {
                    inside = !inside;
                }
            }
            return inside;
        }

        #endregion
    }
}