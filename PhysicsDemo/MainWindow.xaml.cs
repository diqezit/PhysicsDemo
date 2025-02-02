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
    public partial class MainWindow : Window
    {
        // Контейнер в локальной системе координат (центр в (0,0))
        private List<Point> containerPolygon;
        private string selectedShape = "Square";

        // Параметры симуляции
        private double containerAngle = 0;               // угол контейнера (градусы)
        private double rotationSpeed = 30;               // скорость вращения (°/сек)
        private Vector circlePos;                        // положение кружка (мировые координаты)
        private Vector circleVel;                        // скорость кружка (пиксели/сек)
        private readonly double circleRadius = 10;       // радиус кружка (пиксели)
        private readonly Vector gravity = new Vector(0, 500); // гравитация (пиксели/сек^2)
        // Физика столкновений: реституция и трение тангенциальной составляющей
        private readonly double restitution = 0.8;       // коэффициент отскока (0–1)
        private readonly double tangentFriction = 0.9;   // коэффициент затухания тангенциальной составляющей

        private Stopwatch stopwatch;
        private TimeSpan lastTime;

        // Для перетаскивания контейнера
        private bool isDragging = false;
        private Point dragStart;               // позиция мыши при начале перетаскивания
        private Point containerCenter;         // центр контейнера (в мировых координатах)
        private Point containerCenterInitial;  // зафиксированное начальное значение центра при начале перетаскивания

        // Визуальные элементы
        private Path containerPath;
        private Ellipse circleEllipse;

        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            InitShape();
            ResetCircle(); // устанавливаем позицию кружка и контейнера

            // Создаем визуальный элемент для контейнера
            containerPath = new Path
            {
                Stroke = Brushes.DarkSlateGray,
                StrokeThickness = 3,
                SnapsToDevicePixels = true,
                StrokeLineJoin = PenLineJoin.Round
            };
            SimulationCanvas.Children.Add(containerPath);

            // Создаем кружок
            circleEllipse = new Ellipse
            {
                Width = circleRadius * 2,
                Height = circleRadius * 2,
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
            SimulationCanvas.Children.Add(circleEllipse);

            stopwatch = Stopwatch.StartNew();
            lastTime = stopwatch.Elapsed;
            CompositionTarget.Rendering += OnRendering;
        }

        private void ShapeComboBox_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (ShapeComboBox.SelectedItem is System.Windows.Controls.ComboBoxItem item)
            {
                selectedShape = item.Content.ToString();
                InitShape();
                ResetCircle(); // сбрасываем позицию кружка и центра контейнера при смене фигуры
            }
        }

        /// <summary>
        /// Сбрасывает положение и скорость кружка, а также устанавливает центр контейнера.
        /// </summary>
        private void ResetCircle()
        {
            // Если размеры Canvas ещё не вычислены, отложим вызов.
            if (SimulationCanvas == null || SimulationCanvas.ActualWidth == 0 || SimulationCanvas.ActualHeight == 0)
            {
                Dispatcher.BeginInvoke(new Action(ResetCircle), System.Windows.Threading.DispatcherPriority.Loaded);
                return;
            }

            double cx = SimulationCanvas.ActualWidth / 2;
            double cy = SimulationCanvas.ActualHeight / 2;
            containerCenter = new Point(cx, cy);
            circlePos = new Vector(cx, cy - 50); // помещаем кружок внутри контейнера
            circleVel = new Vector(0, 0);
        }

        /// <summary>
        /// Инициализирует вершины контейнера (фигуры) в зависимости от выбранного типа.
        /// </summary>
        private void InitShape()
        {
            containerPolygon = new List<Point>();
            switch (selectedShape)
            {
                case "Square":
                    containerPolygon.Add(new Point(-100, -100));
                    containerPolygon.Add(new Point(100, -100));
                    containerPolygon.Add(new Point(100, 100));
                    containerPolygon.Add(new Point(-100, 100));
                    break;
                case "Triangle":
                    containerPolygon.Add(new Point(0, -120));
                    containerPolygon.Add(new Point(120, 100));
                    containerPolygon.Add(new Point(-120, 100));
                    break;
                case "Pentagon":
                    int sides = 5;
                    double radius = 120;
                    for (int i = 0; i < sides; i++)
                    {
                        double angle = i * 2 * Math.PI / sides - Math.PI / 2;
                        containerPolygon.Add(new Point(radius * Math.Cos(angle), radius * Math.Sin(angle)));
                    }
                    break;
                default:
                    containerPolygon.Add(new Point(-100, -100));
                    containerPolygon.Add(new Point(100, -100));
                    containerPolygon.Add(new Point(100, 100));
                    containerPolygon.Add(new Point(-100, 100));
                    break;
            }
        }

        /// <summary>
        /// Обработчик обновления кадра: обновляет физику и отрисовку.
        /// </summary>
        private void OnRendering(object sender, EventArgs e)
        {
            TimeSpan currentTime = stopwatch.Elapsed;
            double dt = (currentTime - lastTime).TotalSeconds;
            lastTime = currentTime;

            // Обновляем угол контейнера
            containerAngle = (containerAngle + rotationSpeed * dt) % 360;

            // Обновляем физику кружка
            circleVel += gravity * dt;
            circlePos += circleVel * dt;

            // Формируем трансформацию контейнера с использованием переменной containerCenter
            TransformGroup containerTransform = new TransformGroup();
            containerTransform.Children.Add(new RotateTransform(containerAngle));
            containerTransform.Children.Add(new TranslateTransform(containerCenter.X, containerCenter.Y));

            // Преобразуем позицию кружка в локальную систему контейнера для обработки столкновений
            Matrix invMatrix = containerTransform.Value;
            invMatrix.Invert();
            Point localCirclePos = invMatrix.Transform(new Point(circlePos.X, circlePos.Y));
            Matrix invRotate = new RotateTransform(-containerAngle).Value;
            Vector localCircleVel = invRotate.Transform(circleVel);

            // Обрабатываем столкновения с ребрами контейнера
            bool collided = ProcessCollisions(ref localCirclePos, ref localCircleVel);
            if (collided)
            {
                Matrix rotateMatrix = new RotateTransform(containerAngle).Value;
                circleVel = rotateMatrix.Transform(localCircleVel);
                Point correctedWorldPos = containerTransform.Value.Transform(localCirclePos);
                circlePos = new Vector(correctedWorldPos.X, correctedWorldPos.Y);
            }

            // Обновляем отрисовку контейнера
            UpdateContainerVisual(containerTransform);

            // Обновляем позицию кружка на Canvas
            Canvas.SetLeft(circleEllipse, circlePos.X - circleRadius);
            Canvas.SetTop(circleEllipse, circlePos.Y - circleRadius);
        }

        /// <summary>
        /// Обрабатывает столкновения кружка с ребрами контейнера.
        /// Скорость разбивается на нормальную и тангенциальную составляющие.
        /// </summary>
        private bool ProcessCollisions(ref Point localPos, ref Vector localVel)
        {
            bool collided = false;
            foreach (var edge in GetPolygonEdges(containerPolygon))
            {
                Vector edgeVec = edge.Item2 - edge.Item1;
                double edgeLength = edgeVec.Length;
                if (edgeLength == 0) continue;
                Vector edgeDir = edgeVec / edgeLength;
                Vector toCircle = localPos - edge.Item1;
                double proj = Vector.Multiply(toCircle, edgeDir);
                proj = Math.Max(0, Math.Min(edgeLength, proj));
                Point closest = edge.Item1 + edgeDir * proj;
                Vector diff = localPos - closest;
                double distance = diff.Length;
                if (distance < circleRadius)
                {
                    collided = true;
                    Vector normal = diff;
                    if (normal.Length > 0)
                        normal.Normalize();
                    else
                        normal = new Vector(0, -1);

                    // Разбиваем скорость на нормальную и тангенциальную составляющие
                    double vn = Vector.Multiply(localVel, normal);
                    Vector vNormal = vn * normal;
                    Vector vTangent = localVel - vNormal;

                    // Отражаем нормальную составляющую с реституцией и ослабляем тангенциальную
                    double newVn = -restitution * vn;
                    localVel = newVn * normal + vTangent * tangentFriction;

                    // Корректируем положение, чтобы устранить проникновение
                    double penetration = circleRadius - distance;
                    localPos += normal * penetration;
                }
            }
            return collided;
        }

        /// <summary>
        /// Возвращает список ребер полигона в виде пар точек.
        /// </summary>
        private IEnumerable<Tuple<Point, Point>> GetPolygonEdges(List<Point> polygon)
        {
            for (int i = 0; i < polygon.Count; i++)
            {
                yield return new Tuple<Point, Point>(polygon[i], polygon[(i + 1) % polygon.Count]);
            }
        }

        /// <summary>
        /// Обновляет визуальное представление контейнера с учетом заданной трансформации.
        /// </summary>
        private void UpdateContainerVisual(Transform transform)
        {
            PathGeometry geometry = new PathGeometry();
            if (containerPolygon.Count > 0)
            {
                PathFigure figure = new PathFigure
                {
                    StartPoint = transform.Transform(containerPolygon[0]),
                    IsClosed = true,
                    IsFilled = false
                };
                for (int i = 1; i < containerPolygon.Count; i++)
                {
                    figure.Segments.Add(new LineSegment(transform.Transform(containerPolygon[i]), true));
                }
                geometry.Figures.Add(figure);
            }
            containerPath.Data = geometry;
        }

        // --- Обработка перетаскивания контейнера мышью ---

        private void SimulationCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point mousePos = e.GetPosition(SimulationCanvas);
            // Получаем текущую трансформацию контейнера
            TransformGroup containerTransform = new TransformGroup();
            containerTransform.Children.Add(new RotateTransform(containerAngle));
            containerTransform.Children.Add(new TranslateTransform(containerCenter.X, containerCenter.Y));
            List<Point> worldPoly = new List<Point>();
            foreach (var p in containerPolygon)
            {
                worldPoly.Add(containerTransform.Transform(p));
            }
            // Если клик происходит внутри контейнера, начинаем перетаскивание
            if (IsPointInPolygon(mousePos, worldPoly))
            {
                isDragging = true;
                dragStart = mousePos;
                containerCenterInitial = containerCenter;
                SimulationCanvas.CaptureMouse();
            }
        }

        private void SimulationCanvas_MouseMove(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                Point currentPos = e.GetPosition(SimulationCanvas);
                Vector delta = currentPos - dragStart;
                containerCenter = new Point(containerCenterInitial.X + delta.X, containerCenterInitial.Y + delta.Y);
            }
        }

        private void SimulationCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (isDragging)
            {
                isDragging = false;
                SimulationCanvas.ReleaseMouseCapture();
            }
        }

        /// <summary>
        /// Определяет, находится ли точка pt внутри многоугольника polygon (алгоритм ray casting).
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
    }
}
