using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.IO.Ports;

namespace Propagation_d_une_onde_de_pression_dans_un_milieu
{
    class Program
    {
        // grandeur du milieu simulé
        static int screenWidth = 100;
        static int screenHeight = 100;
        static int screenDepth = 100;
        
        // Variables pour le calcul de Q (flux)
        static int length = 1000; // nombre d'itérations
        static float time = 0.01f; // temps en secondes
        static float area = 0.00001f; // aire en m^2
        static float distance = 0.001f; // Distance en m

        // Variables pour le calcul de la gaussienne
        static float compressionCoeff = 20;
        static float compressionDelta = 10f; // Amplitude
        static float t0 = 0.2f; // temps en secondes
        static float c = 1; // sigma^2

        static float lossCoeff = 1;

        // Nombre de cellules enregistreuses
        static int objectSurface = 5;
        // Nombre de cellules ayant des valeurs initiales
        static int impactSurface = 1;

        static float[,,] screen_array = new float[screenWidth, screenHeight, screenDepth];
        static float[,,] flux_array = new float[screenDepth, screenHeight, screenWidth];

        static string[] impact_coords = new string[impactSurface];
        static string[] registering_coords = new string[objectSurface];

        static float[] total_forces = new float[3];

        static float frictionCoeffX = 0.2f;
        static float frictionCoeffY = 1f;
        static float frictionCoeffZ = 1f;

        static float mass = 0.003f; // kg

        static float g = 9.81f; // N

        static float total_desired_movement = 1;

        static SerialPort _serialPort;

        static void StartCommunication()
        {
            _serialPort = new SerialPort("COM3", 9600);
            _serialPort.Open();

            _serialPort.Write("C");
        }

        static void SetCoords()
        {
            impact_coords[0] = "50,25,30";

            registering_coords[0] = "50,25,50";
            registering_coords[1] = "51,25,50";
            registering_coords[2] = "49,25,50";
            registering_coords[3] = "50,25,51";
            registering_coords[4] = "50,25,49";
        }

        static void Main(string[] args)
        {
            //StartCommunication();
            Console.Write("Communication (o/n): ");
            string ans = Console.ReadLine();
            if (ans == "n" || ans == "N") { Environment.Exit(1); }

            Console.Write("Strokes: ");
            string strokes = Console.ReadLine();
            //_serialPort.Write(strokes);

            SetCoords();

            float total_movement = 0f;
            int total_strokes = 0;

            //while(true)
            //{
                total_strokes++;

                for (int i = 0; i < length; i++)
                {
                    Console.WriteLine("Time: " + Math.Round((i + 1) * time, 5).ToString());

                    screen_array = Iterate(i);
                }

                /*
                float[] frictionForces = { frictionCoeffX * mass * g, frictionCoeffY * mass * g, frictionCoeffZ * mass * g };
                float[] resultantForces = { total_forces[0] / length - frictionForces[0], total_forces[1] / length - frictionForces[1], total_forces[2] / length - frictionForces[2] };
                float[] movement = { (resultantForces[0] * (length * time) * (length * time)) / (2 * mass), (resultantForces[1] * (length * time)) * (length * time) / (2 * mass), (resultantForces[2] * (length * time) * (length * time)) / (2 * mass) };

                Console.WriteLine(movement[0]);
                Console.WriteLine(movement[1]);
                Console.WriteLine(movement[2]);

                total_movement += (float)Math.Sqrt(movement[0] * movement[0] + movement[1] * movement[1] + movement[2] * movement[2]);
                if (total_desired_movement <= total_movement) { break; }

                for (int reg = 0; reg < registering_coords.Length; reg++)
                {
                    string[] string_coords = registering_coords[reg].Split(',');
                    float[] new_coords = { (float)Math.Round(int.Parse(string_coords[0]) + movement[0]/distance,0), (float)Math.Round(int.Parse(string_coords[1]) + movement[1]/distance, 0), (float)Math.Round(int.Parse(string_coords[2]) + movement[2]/distance, 0)};
                    registering_coords[reg] = string.Join(",", new_coords);

                    Console.WriteLine(registering_coords[reg]);
                }
                Console.ReadLine();
                
                */
            //}


            Console.WriteLine("Total movement: " + total_movement.ToString());
            Console.WriteLine("Total strokes: " + total_strokes.ToString());
            Console.ReadLine();
        }

        static float[,,] Iterate(int iteration)
        {
            float[,,] update_screen_array = (float[,,])screen_array.Clone();

            float[] current_forces = new float[3];

            for (int i = 0; i < impactSurface; i++)
            {
                string[] string_coords = impact_coords[i].Split(',');

                screen_array[int.Parse(string_coords[0]), int.Parse(string_coords[1]), int.Parse(string_coords[2])] = CalculateImpact(time * iteration);
            }

            for (int z = 0; z < screenDepth; z++)
            {
                for (int y = 0; y < screenHeight; y++)
                {
                    for (int x = 0; x < screenWidth; x++)
                    {
                        // isole la valeur d'un des items
                        float n = screen_array[z, y, x];

                        // Trouve les valeurs adjacentes
                        // Met toutes les valeurs adjacentes = 0 comme valeur par défaut
                        float xm1 = lossCoeff*n;
                        float xp1 = lossCoeff*n;
                        float ym1 = lossCoeff*n;
                        float yp1 = lossCoeff*n;
                        float zm1 = lossCoeff*n;
                        float zp1 = lossCoeff*n;

                        // Regarde si chaque valeur adjacente existe avant d'attribuer sa valeur à une variable
                        if (x - 1 >= 0) { xm1 = screen_array[z, y, x - 1]; }
                        if (x + 1 <= screenWidth - 1) { xp1 = screen_array[z, y, x + 1]; }
                        if (y - 1 >= 0) { ym1 = screen_array[z, y - 1, x]; }
                        if (y + 1 <= screenHeight - 1) { yp1 = screen_array[z, y + 1, x]; }
                        if (z - 1 >= 0) { zm1 = screen_array[z - 1, y, x]; }
                        if (z + 1 <= screenDepth - 1) { zp1 = screen_array[z + 1, y, x]; }

                        // crée une liste de toutes les valeurs adjacentes
                        float[] adj = { xm1, xp1, ym1, yp1, zm1, zp1 };

                        string currentCoords = string.Join(",", new int[] { z, y, x });

                        for (int i = 0; i < registering_coords.Length; i++)
                        {
                            if (registering_coords[i] == currentCoords)
                            {
                                current_forces = RegisterAdjacents(adj, n, current_forces);
                            }
                        }

                        // calcul du flux
                        // passe à travers chaque valeur adjacente
                        for (int value_i = 0; value_i < adj.Length; value_i++)
                        {
                            // ajoute au flux total le flux d'une paire de valeurs
                            flux_array[z, y, x] += ((adj[value_i] - n) * area * time) / distance;
                        }

                        // ajoute le flux à la valeur de la variable n dans une nouvelle liste
                        update_screen_array[z, y, x] += flux_array[z, y, x];
                    }
                }
            }
            Console.WriteLine("Forces (N) (x/y/z): " + Math.Round(current_forces[0], 5).ToString() + " / " + Math.Round(current_forces[1], 5).ToString() + " / " + Math.Round(current_forces[2], 5).ToString() + "\n");

            float[] current_velocities = { current_forces[0]/mass, current_forces[1]/mass, current_forces[2]/mass };

            Console.WriteLine("Velocities (m/s) (x/y/z): " + current_velocities[0] + " / " + current_velocities[1] + " / " + current_velocities[2]);

            total_forces[0] += current_forces[0];
            total_forces[1] += current_forces[1];
            total_forces[2] += current_forces[2];

            return update_screen_array;
        }

        // Retourne la pression en fonction du temps selon une gaussienne
        static float CalculateImpact(float t)
        {
            if (t == 0) { Console.WriteLine("hit"); }

            float x = compressionDelta * (float)Math.Exp((double)(new decimal((-(t - t0) * (t - t0) / c))));
            float pressure = (compressionCoeff * x) / distance;

            return pressure / impactSurface;
        }

        static float[] RegisterAdjacents(float[] adjacents, float n, float[] forces)
        {
            // z, y, x
            float[] pressures = new float[3];
            
            for (int adj = 0; adj < adjacents.Length; adj++)
            {
                // xm1, xp1, ym1, yp1, zm1, zp1
                float pdiff = adjacents[adj] - n;

                switch (adj)
                {
                    case (0):
                        pressures[2] += pdiff; break;
                    case (1):
                        pressures[2] -= pdiff; break;
                    case (2):
                        pressures[1] += pdiff; break;
                    case (3):
                        pressures[1] -= pdiff; break;
                    case (4):
                        pressures[0] += pdiff; break;
                    case (5):
                        pressures[0] -= pdiff; break;
                }
            }

            return new float[] { forces[0] + pressures[0] * area, forces[1] + pressures[1] * area, forces[2] + pressures[2] * area };
        }
    }
}
