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
    /// Основное окно демонстрации физической симуляции.
    /// Реализует 2D‑симуляцию с обнаружением и обработкой коллизий.
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Constants and Structures

        private const double EPSILON = 1e-6;

        /// <summary>
        /// Физические константы, используемые в симуляции.
        /// </summary>
        private readonly struct PhysicsConstants
        {
            public const double CircleRadius = 10;
            public const double Restitution = 0.8;
            public const double TangentFriction = 0.9;
            public const double BaseRotationSpeed = 30;       // Базовая скорость вращения
            public const double RotationSpeedIncrement = 5;     // Изменение скорости при прокрутке
            public const double MaxRotationSpeed = 120;         // Максимальная скорость вращения
            public const double MinRotationSpeed = -120;        // Минимальная скорость (обратная)
            public static readonly Vector Gravity = new Vector(0, 500);
            public const double AirFriction = 0.5;              // Коэффициент сопротивления воздуха (1/с)
        }

        /// <summary>
        /// Состояние симуляции.
        /// </summary>
        private class SimulationState
        {
            /// <summary>
            /// Координаты вершин контейнера.
            /// </summary>
            public List<Point> ContainerPolygon { get; set; }

            /// <summary>
            /// Угол поворота контейнера.
            /// </summary>
            public double ContainerAngle { get; set; }

            /// <summary>
            /// Позиция объекта (шарика).
            /// </summary>
            public Vector CirclePosition { get; set; }

            /// <summary>
            /// Скорость объекта (шарика).
            /// </summary>
            public Vector CircleVelocity { get; set; }

            /// <summary>
            /// Центр контейнера.
            /// </summary>
            public Point ContainerCenter { get; set; }

            /// <summary>
            /// Текущая скорость вращения контейнера.
            /// </summary>
            public double CurrentRotationSpeed { get; set; }

            public SimulationState()
            {
                ContainerPolygon = new List<Point>();
                CurrentRotationSpeed = PhysicsConstants.BaseRotationSpeed;
            }
        }

        #endregion

        #region Private Fields

        private readonly SimulationState _state = new SimulationState();
        private readonly Stopwatch _stopwatch = new Stopwatch();
        private TimeSpan _lastTime;
        private string _selectedShape = "Square";
        private string _selectedBallType = "Обычный шар";

        // Состояние перетаскивания контейнера
        private bool _isDragging;
        private Point _dragStart;
        private Point _containerCenterInitial;

        // UI элементы
        private Path _containerPath;
        private UIElement _ballVisual;

        #endregion

        #region Initialization

        /// <summary>
        /// Конструктор окна.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
        }

        /// <summary>
        /// Обработчик события загрузки окна.
        /// </summary>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            InitializeSimulation();
            InitializeVisuals();
            InitializeEventHandlers();
            StartSimulation();
        }

        /// <summary>
        /// Инициализация состояния симуляции.
        /// </summary>
        private void InitializeSimulation()
        {
            InitShape();
            ResetBall();
        }

        /// <summary>
        /// Инициализация визуальных элементов симуляции.
        /// </summary>
        private void InitializeVisuals()
        {
            _containerPath = CreateContainerPath();
            _ballVisual = CreateBallVisual();
            SimulationCanvas.Children.Clear();
            SimulationCanvas.Children.Add(_containerPath);
            SimulationCanvas.Children.Add(_ballVisual);
        }

        /// <summary>
        /// Инициализация обработчиков событий.
        /// </summary>
        private void InitializeEventHandlers()
        {
            SimulationCanvas.MouseLeftButtonDown += SimulationCanvas_MouseLeftButtonDown;
            SimulationCanvas.MouseMove += SimulationCanvas_MouseMove;
            SimulationCanvas.MouseLeftButtonUp += SimulationCanvas_MouseLeftButtonUp;
            SimulationCanvas.MouseWheel += SimulationCanvas_MouseWheel;
        }

        /// <summary>
        /// Создание визуального элемента контейнера.
        /// </summary>
        private Path CreateContainerPath() => new Path
        {
            Stroke = Brushes.DarkSlateGray,
            StrokeThickness = 3,
            SnapsToDevicePixels = true,
            StrokeLineJoin = PenLineJoin.Round
        };

        /// <summary>
        /// Создание визуального элемента объекта (шар, колесо и т.п.).
        /// </summary>
        private UIElement CreateBallVisual()
        {
            Ellipse ball = new Ellipse
            {
                Width = PhysicsConstants.CircleRadius * 2,
                Height = PhysicsConstants.CircleRadius * 2,
                Effect = new System.Windows.Media.Effects.DropShadowEffect
                {
                    Color = Colors.Black,
                    BlurRadius = 5,
                    ShadowDepth = 2
                }
            };

            // Используем обычное switch-выражение (C# 7.3)
            switch (_selectedBallType)
            {
                case "Колесо":
                    ball.Fill = Brushes.LightGray;
                    ball.Stroke = Brushes.Black;
                    ball.StrokeThickness = 4;
                    break;
                case "Металлический шар":
                    ball.Fill = new RadialGradientBrush(Colors.LightGray, Colors.DarkGray)
                    {
                        GradientOrigin = new Point(0.3, 0.3)
                    };
                    break;
                default: // "Обычный шар"
                    ball.Fill = new RadialGradientBrush(Colors.IndianRed, Colors.DarkRed)
                    {
                        GradientOrigin = new Point(0.3, 0.3)
                    };
                    break;
            }
            return ball;
        }

        /// <summary>
        /// Запуск симуляции.
        /// </summary>
        private void StartSimulation()
        {
            _stopwatch.Start();
            _lastTime = _stopwatch.Elapsed;
            CompositionTarget.Rendering += OnRendering;
        }

        #endregion

        #region Helper Methods

        /// <summary>
        /// Ограничивает значение в заданном диапазоне.
        /// </summary>
        private double ClampValue(double value, double min, double max) =>
            value < min ? min : value > max ? max : value;

        #endregion

        #region Shape Management

        /// <summary>
        /// Обработчик изменения выбранной фигуры.
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
        /// Обработчик изменения выбранного типа объекта.
        /// </summary>
        private void BallTypeComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (SimulationCanvas == null)
                return;

            if (BallTypeComboBox.SelectedItem is ComboBoxItem item)
            {
                _selectedBallType = item.Content.ToString();
                if (_ballVisual != null)
                    SimulationCanvas.Children.Remove(_ballVisual);
                _ballVisual = CreateBallVisual();
                SimulationCanvas.Children.Add(_ballVisual);
                ResetBall();
            }
        }

        /// <summary>
        /// Сброс позиции и скорости объекта.
        /// </summary>
        private void ResetBall()
        {
            if (!IsCanvasReady())
            {
                Dispatcher.BeginInvoke(new Action(ResetBall),
                    System.Windows.Threading.DispatcherPriority.Loaded);
                return;
            }

            double cx = SimulationCanvas.ActualWidth / 2;
            double cy = SimulationCanvas.ActualHeight / 2;
            _state.ContainerCenter = new Point(cx, cy);
            _state.CirclePosition = new Vector(cx, cy - 50);
            _state.CircleVelocity = new Vector(0, 0);
        }

        /// <summary>
        /// Проверка готовности канвы для отрисовки.
        /// </summary>
        private bool IsCanvasReady() =>
            SimulationCanvas != null && SimulationCanvas.ActualWidth > 0 && SimulationCanvas.ActualHeight > 0;

        /// <summary>
        /// Инициализация выбранной формы контейнера.
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
                    CreateRegularPolygon(5, 120, -Math.PI / 2);
                    break;
                case "Hexagon":
                    CreateRegularPolygon(6, 120, 0);
                    break;
                case "Octagon":
                    CreateRegularPolygon(8, 120, 0);
                    break;
                default:
                    CreateSquare();
                    break;
            }
        }

        /// <summary>
        /// Создаёт квадрат.
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
        /// Создаёт треугольник.
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
        /// Создаёт правильный многоугольник с заданным числом сторон, радиусом и начальным углом.
        /// </summary>
        private void CreateRegularPolygon(int sides, double radius, double startAngle)
        {
            for (int i = 0; i < sides; i++)
            {
                double angle = startAngle + i * 2 * Math.PI / sides;
                _state.ContainerPolygon.Add(new Point(
                    radius * Math.Cos(angle),
                    radius * Math.Sin(angle)));
            }
        }

        #endregion

        #region Physics Simulation

        /// <summary>
        /// Основной метод обновления физики и отрисовки.
        /// </summary>
        private void OnRendering(object sender, EventArgs e)
        {
            double dt = CalculateDeltaTime();
            UpdatePhysics(dt);
            UpdateVisuals();
        }

        /// <summary>
        /// Вычисляет временной интервал между кадрами.
        /// </summary>
        private double CalculateDeltaTime()
        {
            TimeSpan currentTime = _stopwatch.Elapsed;
            double dt = (currentTime - _lastTime).TotalSeconds;
            _lastTime = currentTime;
            return dt;
        }

        /// <summary>
        /// Обновляет физическое состояние симуляции с использованием субшагов и итеративного разрешения коллизий.
        /// </summary>
        private void UpdatePhysics(double dt)
        {
            double fixedDt = 1.0 / 300.0;
            while (dt > 0)
            {
                double step = Math.Min(fixedDt, dt);
                dt -= step;

                _state.ContainerAngle = (_state.ContainerAngle + _state.CurrentRotationSpeed * step) % 360;
                _state.CircleVelocity += PhysicsConstants.Gravity * step;
                _state.CircleVelocity *= (1 - PhysicsConstants.AirFriction * step);
                _state.CirclePosition += _state.CircleVelocity * step;

                var containerTransform = CreateContainerTransform();
                (Point localPos, Vector localVel) = TransformToLocal(_state.CirclePosition, _state.CircleVelocity, containerTransform);

                int iterations = 0;
                bool collisionDetected;
                do
                {
                    collisionDetected = ProcessCollisions(ref localPos, ref localVel);
                    iterations++;
                }
                while (collisionDetected && iterations < 10);

                (Vector worldVel, Point worldPos) = TransformToWorld(localVel, localPos, containerTransform);
                _state.CircleVelocity = worldVel;
                _state.CirclePosition = new Vector(worldPos.X, worldPos.Y);
            }
        }

        #endregion

        #region Transformation Helpers

        /// <summary>
        /// Создаёт преобразование контейнера.
        /// </summary>
        private TransformGroup CreateContainerTransform()
        {
            var transform = new TransformGroup();
            transform.Children.Add(new RotateTransform(_state.ContainerAngle));
            transform.Children.Add(new TranslateTransform(_state.ContainerCenter.X, _state.ContainerCenter.Y));
            return transform;
        }

        /// <summary>
        /// Преобразует мировые координаты в локальные.
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
        /// Преобразует локальные координаты в мировые.
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
        /// Обрабатывает коллизии между объектом и сторонами контейнера.
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
        /// Обрабатывает коллизию с одной стороной контейнера.
        /// </summary>
        private bool ProcessEdgeCollision(Tuple<Point, Point> edge, ref Point localPos, ref Vector localVel)
        {
            Vector edgeVec = edge.Item2 - edge.Item1;
            double edgeLength = edgeVec.Length;
            if (edgeLength < EPSILON)
                return false;

            Vector edgeDir = edgeVec / edgeLength;
            Vector toBall = localPos - edge.Item1;
            double proj = Math.Max(0, Math.Min(edgeLength, Vector.Multiply(toBall, edgeDir)));
            Point closest = edge.Item1 + edgeDir * proj;
            Vector diff = localPos - closest;
            double distance = diff.Length;

            if (distance >= PhysicsConstants.CircleRadius)
                return false;

            Vector normal = diff.Length > EPSILON ? diff / diff.Length : new Vector(0, -1);
            double vn = Vector.Multiply(localVel, normal);
            if (vn < 0)
            {
                double impulseMag = -(1 + PhysicsConstants.Restitution) * vn;
                Vector impulse = impulseMag * normal;
                localVel += impulse;
                Vector vNormal = (Vector.Multiply(localVel, normal)) * normal;
                Vector vTangent = localVel - vNormal;
                localVel = vNormal + vTangent * PhysicsConstants.TangentFriction;
            }
            localPos += normal * (PhysicsConstants.CircleRadius - distance + 0.5);
            return true;
        }

        /// <summary>
        /// Возвращает стороны многоугольника в виде пар точек.
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

        #region Visual Updates

        /// <summary>
        /// Обновляет визуальное представление симуляции.
        /// </summary>
        private void UpdateVisuals()
        {
            var containerTransform = CreateContainerTransform();
            UpdateContainerVisual(containerTransform);
            UpdateBallPosition();
        }

        /// <summary>
        /// Обновляет визуальное представление контейнера.
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
        /// Обновляет позицию объекта на канве.
        /// </summary>
        private void UpdateBallPosition()
        {
            Canvas.SetLeft(_ballVisual, _state.CirclePosition.X - PhysicsConstants.CircleRadius);
            Canvas.SetTop(_ballVisual, _state.CirclePosition.Y - PhysicsConstants.CircleRadius);
        }

        #endregion

        #region Mouse Event Handlers

        /// <summary>
        /// Обрабатывает событие прокрутки мыши для изменения скорости вращения контейнера.
        /// </summary>
        private void SimulationCanvas_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            double speedChange = e.Delta > 0 ? PhysicsConstants.RotationSpeedIncrement : -PhysicsConstants.RotationSpeedIncrement;
            _state.CurrentRotationSpeed = ClampValue(
                _state.CurrentRotationSpeed + speedChange,
                PhysicsConstants.MinRotationSpeed,
                PhysicsConstants.MaxRotationSpeed);
        }

        /// <summary>
        /// Обрабатывает нажатие левой кнопки мыши для начала перетаскивания контейнера.
        /// </summary>
        private void SimulationCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point mousePos = e.GetPosition(SimulationCanvas);
            var worldPolygon = GetWorldPolygon();
            if (IsPointInPolygon(mousePos, worldPolygon))
                StartDragging(mousePos);
        }

        /// <summary>
        /// Возвращает вершины контейнера в мировых координатах.
        /// </summary>
        private List<Point> GetWorldPolygon()
        {
            var containerTransform = CreateContainerTransform();
            return _state.ContainerPolygon.ConvertAll(p => containerTransform.Transform(p));
        }

        /// <summary>
        /// Начинает операцию перетаскивания контейнера.
        /// </summary>
        private void StartDragging(Point mousePos)
        {
            _isDragging = true;
            _dragStart = mousePos;
            _containerCenterInitial = _state.ContainerCenter;
            SimulationCanvas.CaptureMouse();
        }

        /// <summary>
        /// Обрабатывает перемещение мыши во время перетаскивания.
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
        /// Завершает операцию перетаскивания контейнера.
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
        /// Определяет, находится ли точка внутри многоугольника (алгоритм лучевого броска).
        /// </summary>
        private bool IsPointInPolygon(Point pt, List<Point> polygon)
        {
            bool inside = false;
            for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            {
                if (((polygon[i].Y > pt.Y) != (polygon[j].Y > pt.Y)) &&
                    (pt.X < (polygon[j].X - polygon[i].X) * (pt.Y - polygon[i].Y) / (polygon[j].Y - polygon[i].Y) + polygon[i].X))
                {
                    inside = !inside;
                }
            }
            return inside;
        }

        #endregion
    }
}
