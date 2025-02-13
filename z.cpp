#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <cstdint>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <algorithm>
#include <queue>
//Berintsev Vyacheslav 212 group
class Logger {
    std::ofstream server_log;
    std::ofstream client_log;
    bool log_to_file;

public:
    Logger() : log_to_file(true) {}

    ~Logger() {
        if (server_log.is_open()) server_log.close();
        if (client_log.is_open()) client_log.close();
    }

    void configure(bool enable_logging, std::string server_filename, std::string client_filename) {
        server_filename = trim_quotes(server_filename);
        client_filename = trim_quotes(client_filename);

        if (server_log.is_open()) server_log.close();
        if (client_log.is_open()) client_log.close();

        log_to_file = enable_logging;

        if (log_to_file) {
            server_log.open(server_filename, std::ios::app);
            client_log.open(client_filename, std::ios::app);

            if (!server_log.is_open()) {
                std::cerr << "Error: Unable to open server log file: " << server_filename << std::endl;
                log_to_file = false;
            }
            if (!client_log.is_open()) {
                std::cerr << "Error: Unable to open client log file: " << client_filename << std::endl;
                log_to_file = false;
            }
        }
    }

    void log_server(const std::string& message) {
        if (log_to_file && server_log.is_open()) {
            server_log << current_time() << " - " << message << std::endl;
        }
        std::cout << current_time() << " - " << message << std::endl;
    }

    void log_client(const std::string& message) {
        if (log_to_file && client_log.is_open()) {
            client_log << current_time() << " - " << message << std::endl;
        }
        std::cout << current_time() << " - " << message << std::endl;
    }

    std::string current_time() const {
        std::time_t t = std::time(nullptr);
        std::tm* now = std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(now, "%d.%m.%Y %H:%M:%S");
        return oss.str();
    }

private:
    std::string trim_quotes(const std::string& str) {
        if (str.size() >= 2 && str.front() == '"' && str.back() == '"') {
            return str.substr(1, str.size() - 2);
        }
        return str;
    }
};

Logger logger;
//Berintsev Vyacheslav 212 group
class Config {
public:
    int default_length = 100;
    int default_width = 100;
    double default_h = 1.0;
    double default_x0 = 50.0;
    double default_y0 = 50.0;
    double default_sigma_x = 1.0;
    double default_sigma_y = 1.0;
    bool enable_file_logging = true;
    std::string server_log_filename = "server_log.txt";
    std::string client_log_filename = "client_log.txt";
    std::string commands_filename = "";

    bool load(const std::string& filename) {
        std::ifstream config_file(filename);
        if (!config_file.is_open()) {
            std::cerr << "Error: failed to open config file " << filename << std::endl;
            return false;
        }

        std::string key;
        while (config_file >> key) {
            if (key == "init_length") config_file >> default_length;
            else if (key == "init_width") config_file >> default_width;
            else if (key == "gauss_h") config_file >> default_h;
            else if (key == "gauss_x0") config_file >> default_x0;
            else if (key == "gauss_y0") config_file >> default_y0;
            else if (key == "gauss_sigma_x") config_file >> default_sigma_x;
            else if (key == "gauss_sigma_y") config_file >> default_sigma_y;
            else if (key == "enable_file_logging") {
                std::string val;
                config_file >> val;
                enable_file_logging = (val == "true" || val == "1");
            }
            else if (key == "server_log_filename") {
                config_file >> server_log_filename;
                server_log_filename = trim_quotes(server_log_filename);
            }
            else if (key == "client_log_filename") {
                config_file >> client_log_filename;
                client_log_filename = trim_quotes(client_log_filename);
            }
            else if (key == "commands_filename") {
                config_file >> commands_filename;
                commands_filename = trim_quotes(commands_filename);
            } else std::getline(config_file, key);
        }
        config_file.close();
        return true;
    }

private:
    std::string trim_quotes(const std::string& str) {
        if (str.size() >= 2 && str.front() == '"' && str.back() == '"') {
            return str.substr(1, str.size() - 2);
        }
        return str;
    }
};
//Berintsev Vyacheslav 212 group
class Field {
public:
    int length, width;
    std::vector<std::vector<double>> matrix;
    std::vector<int> assignments;
    Field(int l, int w) : length(l), width(w) {
        matrix.resize(length, std::vector<double>(width, 0));
        logger.log_client("Interface received 'init' and pass to Control");
        logger.log_server("Control received 'init' and pass to Field");
        logger.log_server("Field created with size " + std::to_string(length) + "x" + std::to_string(width));
    }

    // Конструктор копирования
    Field(const Field& other) : length(other.length), width(other.width), matrix(other.matrix) {
        
    }

    void apply_binary_threshold(double threshold) {
        for (int i = 0; i < length; ++i) {
            for (int j = 0; j < width; ++j) {
                matrix[i][j] = (matrix[i][j] > threshold) ? 1.0 : 0.0;
            }
        }
        logger.log_client("Interface received BIN and pass to Control");
        logger.log_server("Control received BIN and pass to Field");
        logger.log_server("BIN cut with: " + std::to_string(threshold));
    }

    void load_from_bmp(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            logger.log_client("Error: could not open BMP file " + filename);
            std::cerr << "Error: could not open BMP file " << filename << std::endl;
            return;
        }

        unsigned char bmp_header[54];
        file.read(reinterpret_cast<char*>(bmp_header), 54);

        int bmp_width = bmp_header[18] | (bmp_header[19] << 8) | (bmp_header[20] << 16) | (bmp_header[21] << 24);
        int bmp_height = bmp_header[22] | (bmp_header[23] << 8) | (bmp_header[24] << 16) | (bmp_header[25] << 24);

        if (bmp_width != width || bmp_height != length) {
            logger.log_server("Error: BMP file dimensions do not match field size.");
            std::cerr << "Error: BMP file dimensions do not match field size." << std::endl;
            return;
        }

        const int PIXEL_SIZE = 3;
        for (int i = length - 1; i >= 0; --i) {
            for (int j = 0; j < width; ++j) {
                unsigned char pixel[3];
                file.read(reinterpret_cast<char*>(pixel), PIXEL_SIZE);
                int grayscale = (static_cast<int>(pixel[0]) + pixel[1] + pixel[2]) / 3;
                matrix[i][j] = grayscale;
            }
        }
        file.close();
        logger.log_server("Field loaded from file: " + filename);
    }

    void save_to_gnuplot(const std::string& filename) const {
        std::ofstream out(filename);
        for (int i = 0; i < length; ++i) {
            for (int j = 0; j < width; ++j) {
                out << i << " " << j << " " << matrix[i][j] << std::endl;
            }
            out << std::endl;
        }
        out.close();
        logger.log_server("Field saved to Gnuplot file: " + filename);
    }

    void save_to_bmp(const std::string& filename) const {
    const int BMP_HEADER_SIZE = 54;
    const int PIXEL_SIZE = 3;
    int file_size = BMP_HEADER_SIZE + PIXEL_SIZE * length * width;

    unsigned char bmp_header[BMP_HEADER_SIZE] = { 0 };
    bmp_header[0] = 'B';
    bmp_header[1] = 'M';
    bmp_header[2] = file_size & 0xFF;
    bmp_header[3] = (file_size >> 8) & 0xFF;
    bmp_header[4] = (file_size >> 16) & 0xFF;
    bmp_header[5] = (file_size >> 24) & 0xFF;
    bmp_header[10] = BMP_HEADER_SIZE;

    bmp_header[14] = 40;
    bmp_header[18] = width & 0xFF;
    bmp_header[19] = (width >> 8) & 0xFF;
    bmp_header[20] = (width >> 16) & 0xFF;
    bmp_header[21] = (width >> 24) & 0xFF;
    bmp_header[22] = length & 0xFF;
    bmp_header[23] = (length >> 8) & 0xFF;
    bmp_header[24] = (length >> 16) & 0xFF;
    bmp_header[25] = (length >> 24) & 0xFF;
    bmp_header[26] = 1;
    bmp_header[28] = 24;

    std::ofstream out(filename, std::ios::binary);
    out.write(reinterpret_cast<char*>(bmp_header), BMP_HEADER_SIZE);

    for (int i = length - 1; i >= 0; --i) {
        for (int j = 0; j < width; ++j) {
            // Масштабируем бинарные значения в диапазон [0, 255]
            int value = (matrix[i][j] > 0.5) ? 255 : 0;
            unsigned char pixel[3] = { static_cast<unsigned char>(value),
                                       static_cast<unsigned char>(value),
                                       static_cast<unsigned char>(value) };
            out.write(reinterpret_cast<char*>(pixel), PIXEL_SIZE);
        }
    }
    out.close();

    
    
}
//berintsev Vyacheslav 212 
// K-means кластеризация
std::vector<std::vector<std::pair<double, double>>> kMeansClustering(int k, int p) {
    if (matrix.empty()) {
        throw std::runtime_error("Field matrix is empty.");
    }

    assignments.resize(length * width, -1);
    std::vector<std::pair<double, double>> centers(k);

    // Инициализация центроид
    srand(static_cast<unsigned>(time(0)));
    for (int i = 0; i < k; ++i) {
        int x, y;
        do {
            x = rand() % length;
            y = rand() % width;
        } while (matrix[x][y] <= 0.5);
        centers[i] = {x, y};
    }

    // Основной цикл кластеризации
    bool converged = false;
    while (!converged) {
        converged = true;

        // Присвоение точек кластерам
        for (int i = 0; i < length; ++i) {
            for (int j = 0; j < width; ++j) {
                if (matrix[i][j] <= 0.5) continue;

                int best_cluster = 0;
                double best_distance = std::numeric_limits<double>::max();

                for (int c = 0; c < k; ++c) {
                    double dist = std::pow(i - centers[c].first, 2) + std::pow(j - centers[c].second, 2);
                    if (dist < best_distance) {
                        best_distance = dist;
                        best_cluster = c;
                    }
                }

                int index = i * width + j;
                if (assignments[index] != best_cluster) {
                    converged = false;
                    assignments[index] = best_cluster;
                }
            }
        }

        // Пересчет центроид
        for (int c = 0; c < k; ++c) {
            double sum_x = 0, sum_y = 0;
            int count = 0;

            for (int i = 0; i < length; ++i) {
                for (int j = 0; j < width; ++j) {
                    int index = i * width + j;
                    if (assignments[index] == c) {
                        sum_x += i;
                        sum_y += j;
                        count++;
                    }
                }
            }

            if (count > 0) {
                centers[c] = {sum_x / count, sum_y / count};
            }
        }
    }

    // Поиск p центроид внутри каждого кластера
    std::vector<std::vector<std::pair<double, double>>> cluster_centroids(k);

    for (int c = 0; c < k; ++c) {
        std::vector<std::pair<int, int>> cluster_points;

        for (int i = 0; i < length; ++i) {
            for (int j = 0; j < width; ++j) {
                int index = i * width + j;
                if (assignments[index] == c) {
                    cluster_points.push_back({i, j});
                }
            }
        }

        // Инициализация p центроид
        std::vector<std::pair<double, double>> sub_centroids(p);
        for (int i = 0; i < p; ++i) {
            int idx = rand() % cluster_points.size();
            sub_centroids[i] = {cluster_points[idx].first, cluster_points[idx].second};
        }

        // Локальная кластеризация внутри кластера
        bool sub_converged = false;
        std::vector<int> sub_assignments(cluster_points.size(), -1);

        while (!sub_converged) {
            sub_converged = true;

            // Присвоение точек p центроидам
            for (size_t i = 0; i < cluster_points.size(); ++i) {
                double best_distance = std::numeric_limits<double>::max();
                int best_centroid = 0;

                for (int j = 0; j < p; ++j) {
                    double dist = std::pow(cluster_points[i].first - sub_centroids[j].first, 2) +
                                  std::pow(cluster_points[i].second - sub_centroids[j].second, 2);
                    if (dist < best_distance) {
                        best_distance = dist;
                        best_centroid = j;
                    }
                }

                if (sub_assignments[i] != best_centroid) {
                    sub_converged = false;
                    sub_assignments[i] = best_centroid;
                }
            }

            // Пересчет субцентроид
            std::vector<std::pair<double, double>> new_sub_centroids(p, {0, 0});
            std::vector<int> sub_counts(p, 0);

            for (size_t i = 0; i < cluster_points.size(); ++i) {
                int centroid_idx = sub_assignments[i];
                new_sub_centroids[centroid_idx].first += cluster_points[i].first;
                new_sub_centroids[centroid_idx].second += cluster_points[i].second;
                sub_counts[centroid_idx]++;
            }

            for (int j = 0; j < p; ++j) {
                if (sub_counts[j] > 0) {
                    new_sub_centroids[j].first /= sub_counts[j];
                    new_sub_centroids[j].second /= sub_counts[j];
                }
            }

            sub_centroids = new_sub_centroids;
        }

        cluster_centroids[c] = sub_centroids;
    }

    // Вывод результата
    for (int c = 0; c < k; ++c) {
        std::cout << "Cluster " << c + 1 << ":" << std::endl;
        for (const auto& centroid : cluster_centroids[c]) {
            std::cout << "(" << centroid.first << ", " << centroid.second << ")" << std::endl;
              
        }
    }

    return cluster_centroids;
}


};
//Berintsev Vyacheslav 212 group
class WaveAnalyzer {
public:
    static void analyze(Field& field) {
        int length = field.length;
        int width = field.width;
        auto& matrix = field.matrix;

        std::vector<std::vector<bool>> visited(length, std::vector<bool>(width, false));
        int component_count = 0;

        for (int i = 0; i < length; ++i) {
            for (int j = 0; j < width; ++j) {
                // Используем проверку на значение > 0.5 для бинарной матрицы
                if (matrix[i][j] > 0.5 && !visited[i][j]) {
                    ComponentData component = perform_wave(matrix, visited, i, j);
                    component_count++;

                    std::cout << "Component " << component_count << ": "
                              << "Center (" << component.center_x << ", " << component.center_y << "), "
                              << "Square (" << component.min_x << ", " << component.min_y << ") to (" 
                              << component.max_x << ", " << component.max_y << "), "
                              << "Size: " << component.size << std::endl;

                  //  logger.log_server("Found component " + std::to_string(component_count) +
                                     // ": Center (" + std::to_string(component.center_x) + ", " + 
                                     // std::to_string(component.center_y) + "), Size " +
                                    //  std::to_string(component.size));
                }
            }
        }

        std::cout << "Total components: " << component_count << std::endl;
        logger.log_client("Interface received wave and pass to Control");
        logger.log_server("Control received wave and pass to Wave");
        logger.log_server("Wave count: " + std::to_string(component_count) + " components.");
        
    }

private:
    struct ComponentData {
        int min_x, min_y, max_x, max_y;
        double center_x, center_y;
        int size;
    };

    static ComponentData perform_wave(std::vector<std::vector<double>>& matrix,
                                      std::vector<std::vector<bool>>& visited,
                                      int start_x, int start_y) {
        int length = matrix.size();
        int width = matrix[0].size();
        std::queue<std::pair<int, int>> queue;
        queue.push(std::make_pair(start_x, start_y));
        visited[start_x][start_y] = true;

        int min_x = start_x, max_x = start_x;
        int min_y = start_y, max_y = start_y;
        int size = 0;
        double sum_x = 0, sum_y = 0;

        const std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

        while (!queue.empty()) {
            std::pair<int, int> current = queue.front();
            queue.pop();
            int x = current.first;
            int y = current.second;

            size++;
            sum_x += x;
            sum_y += y;
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);

            for (size_t i = 0; i < directions.size(); ++i) {
                int dx = directions[i].first;
                int dy = directions[i].second;
                int nx = x + dx;
                int ny = y + dy;

                if (nx >= 0 && nx < length && ny >= 0 && ny < width &&
                    matrix[nx][ny] > 0.5 && !visited[nx][ny]) {
                    queue.push(std::make_pair(nx, ny));
                    visited[nx][ny] = true;
                }
            }
        }

        return {min_x, min_y, max_x, max_y, sum_x / size, sum_y / size, size};
    }
};



//Berintsev Vyacheslav 212 group
class Gauss {
public:
    double h, x0, y0, sigma_x, sigma_y;

    Gauss(double h, double x0, double y0, double sigma_x, double sigma_y)
        : h(h), x0(x0), y0(y0), sigma_x(sigma_x), sigma_y(sigma_y) {}

   bool validate_sigma(double sigma) const {
        const double EPSILON = 1e-9;
        return sigma > EPSILON && sigma >= 0.5 && sigma <= 5.0;
    }

    void apply(Field& field) const {
        const double EPSILON = 1e-9; 
        if (!validate_sigma(sigma_x) || !validate_sigma(sigma_y)) {
            logger.log_server("Error: invalid sigma value.");
            std::cerr << "Error: invalid sigma value." << std::endl;
            return;
        }

    
    
        for (int i = 0; i < field.length; ++i) {
            for (int j = 0; j < field.width; ++j) {
                double dx = i - x0;
                double dy = j - y0;
                
                double denom_x = 2*sigma_x*sigma_x;
                double denom_y = 2*sigma_y*sigma_y;
                double den_x = 3.5 * denom_x;
                double den_y = 3.5 * denom_y;
                if (denom_x < EPSILON || denom_y < EPSILON) {
                logger.log_server("Error: denominator is too small.");
                continue; 
                }
                double exponent = -((dx *dx) / den_x + (dy*dy) / den_y); 
                if (exponent < -700) {
                    exponent = -700; 
                }
                
                field.matrix[i][j] += h * exp(exponent);
            }
        }
        logger.log_server("Gauss applied to Field");
}
};
//Berintsev Vyacheslav 212 group
class Control {
    std::vector<Gauss> gausses;
    Field* field = nullptr;

public:
    Field* get_field() { return field; }

    void init(int length, int width) {
        if (field) {
            logger.log_client("Interface received attempted reinitialization and pass to Control");
            logger.log_server("Control received attempted reinitialization but field already exists.");
            return;
        }
        field = new Field(length, width);
    }

    void add_gauss(double h, double x0, double y0, double sigma_x, double sigma_y) {
        if (!field) {
            logger.log_client("Error: field not initialized for Gauss addition.");
            return;
        }
        logger.log_client("Interface received 'g' and pass to Control");

        Gauss g(h, x0, y0, sigma_x, sigma_y);
        g.apply(*field);
    }

    void generate() {
        if (!field) {
            std::cerr << "Error! Field not initialized!" << std::endl;
            logger.log_server("Control received 'generate', but field was not initialized.");
            return;
        }
        logger.log_client("Interface received 'generate' and pass to Control");
        logger.log_server("Control passed 'generate' command to Field");

        for (size_t i = 0; i < gausses.size(); ++i) {
            logger.log_server("Field is applying gauss #" + std::to_string(i + 1) + " to Field");
            gausses[i].apply(*field);
        }

        logger.log_server("Field completed applying all gausses");
    }

    void save_bmp(const std::string& filename) {
        if (!field) {
            logger.log_client("Error: field not initialized for saving.");
            return;
        }
        field->save_to_bmp(filename);
    }

    void gnuplot(const std::string& filename) {
        if (!field) {
            std::cerr << "Error! Field not initialized!" << std::endl;
            logger.log_server("Control received 'gnuplot', but field was not initialized.");
            return;
        }

        field->save_to_gnuplot("gnuplot_" + filename + ".txt");
        std::ofstream gp_file("gnuplot_commands.txt");
        gp_file << "set view 60,30\n";
        gp_file << "set palette defined (0 \"blue\", 1 \"red\")\n";
        gp_file << "set pm3d at s\n";
        gp_file << "splot 'gnuplot_" << filename << ".txt' with lines\n";
        gp_file.close();
        logger.log_server("Field generated Gnuplot file: " + filename);
    }
     void apply_bin(double threshold) {
        if (!field) {
            std::cerr << "Error: Field not initialized." << std::endl;
            return;
        }
        Field binarized_field = *field;
        binarized_field.apply_binary_threshold(threshold);
        binarized_field.save_to_bmp("bin_cut.bmp");

        std::cout << "Cut field with " << threshold << " and save in bin_cut.bmp" << std::endl;
    }
    void wave_analysis(const std::string& filename) {
        if (!field) {
            std::cerr << "Error: Field not initialized." << std::endl;
            return;
        }
        field->load_from_bmp(filename);
        WaveAnalyzer::analyze(*field);
    }

    void load_bmp(const std::string& filename) {
        if (!field) {
            std::cerr << "Error: Field not initialized." << std::endl;
            return;
        }
        field->load_from_bmp(filename);
        std::cout << "Field loaded from BMP file: " << filename << std::endl;
    }
     void group(int k, int p) {
    if (!field) {
        throw std::runtime_error("Field not initialized.");
    }
    auto centers = field->kMeansClustering(k, p);
    std::cout << "Clustering completed with " << k << " clusters and " << p << " centroids per cluster." << std::endl;
    logger.log_client("Interface received 'group' and pass to Control");
        logger.log_server("Control received 'group' and pass to Field");
             logger.log_server("Field completed k-means");
}
};
//Berintsev Vyacheslav 212 group
class Interface {
    Control control;
    Config config;

public:
    void run() {
        std::string config_filename;
        std::cout << "Enter configuration file name: ";
        std::cin >> config_filename;

        if (!config.load(config_filename)) {
            std::cout << "Error loading configuration file. Using default values." << std::endl;
            logger.log_client("Error loading configuration file. Using default values.");
        }

        logger.configure(config.enable_file_logging, config.server_log_filename, config.client_log_filename);

        std::string input_source;
        std::cout << "Select input source (keyboard/file): ";
        std::cin >> input_source;

        if (input_source == "file") {
            std::string commands_filename = config.commands_filename;
            if (commands_filename.empty()) {
                std::cout << "Enter commands file name: ";
                std::cin >> commands_filename;
            }
            std::ifstream commands_file(commands_filename);
            if (!commands_file.is_open()) {
                logger.log_client("Error opening commands file: " + commands_filename);
                std::cerr << "Error opening commands file: " << commands_filename << std::endl;
                return;
            }
            logger.log_client("Reading commands from file: " + commands_filename);
            std::string command_line;
            while (std::getline(commands_file, command_line)) {
                execute_command(command_line);
            }
            commands_file.close();
        } else if (input_source == "keyboard") {
            std::cin.ignore();
            while (true) {
                std::cout << "> ";
                std::string command_line;
                std::getline(std::cin, command_line);

                if (command_line == "exit") {
                    std::cout << "Exiting the program." << std::endl;
                    logger.log_server("Exiting the program.");
                    break;
                }

                execute_command(command_line);
            }
        } else {
            std::cerr << "Invalid input source selection. Please choose either 'keyboard' or 'file'." << std::endl;
            logger.log_client("Invalid input source selection.");
        }
    }

private:
    void execute_command(const std::string& command_line) {
        std::istringstream iss(command_line);
        std::string command;
        iss >> command;

        if (command == "init") {
            int length = config.default_length;
            int width = config.default_width;

            if (!(iss >> length)) length = config.default_length;
            if (!(iss >> width)) width = config.default_width;

            control.init(length, width);
            std::cout << "Field initialized with size " << length << "x" << width << std::endl;
        }
        else if (command == "g") {
            double h = config.default_h;
            double x0 = config.default_x0;
            double y0 = config.default_y0;
            double sigma_x = config.default_sigma_x;
            double sigma_y = config.default_sigma_y;

            if (!(iss >> h)) h = config.default_h;
            if (!(iss >> x0)) x0 = config.default_x0;
            if (!(iss >> y0)) y0 = config.default_y0;
            if (!(iss >> sigma_x)) sigma_x = config.default_sigma_x;
            if (!(iss >> sigma_y)) sigma_y = config.default_sigma_y;

            control.add_gauss(h, x0, y0, sigma_x, sigma_y);
            std::cout << "Added Gauss with parameters (h=" << h << ", x0=" << x0
                      << ", y0=" << y0 << ", sigma_x=" << sigma_x << ", sigma_y=" << sigma_y << ")" << std::endl;
        }
        else if (command == "BIN") {
            double threshold;
            if (!(iss >> threshold)) {
                std::cerr << "POrog ne ukazan" << std::endl;
                return;
            }

              control.apply_bin(threshold);
            }
        else if (command == "generate") {
            control.generate();
            std::cout << "Generated field with applied Gaussians" << std::endl;
        }
          else if (command == "group") {
    int k, p;
    if (!(iss >> k) || k <= 0 || !(iss >> p) || p <= 0) {
        std::cerr << "Invalid number of clusters or centroids per cluster." << std::endl;
        return;
    }
    control.group(k, p);
}

        else if (command == "gnuplot") {
            std::string filename;
            if (!(iss >> filename)) filename = "default";
            control.gnuplot(filename);
            std::cout << "Field saved to Gnuplot file: " << filename << std::endl;
        }
        else if (command == "save") {
            std::string filetype;
            iss >> filetype;
            if (filetype == "bmp") {
                std::string filename;
                if (!(iss >> filename)) filename = "bmp_538.bmp";
                control.save_bmp(filename);
                std::cout << "Field saved to BMP file: " << filename << std::endl;
                logger.log_client("Interface received 'save bmp' and pass to Control");
        logger.log_server("Control received 'save bmp' and pass to Field");
        logger.log_server("Field completed save to BMP file: " + filename);
            }
        }
        else if (command == "load") {
            std::string filetype;
            iss >> filetype;
            if (filetype == "bmp") {
                std::string filename;
                if (!(iss >> filename)) {
                    std::cerr << "Error: BMP filename is required" << std::endl;
                    return;
                }
                  control.load_bmp(filename);
            }
        }
        else if (command == "wave") {
            std::string filename;
            if (!(iss >> filename)) {
                std::cerr << "Error: No BMP file specified for wave analysis." << std::endl;
                return;
            }

             control.wave_analysis(filename);
        }
        else {
            std::cout << "Unknown command" << std::endl;
            logger.log_client("Received unknown command: " + command_line);
        }
    }
};
//Berintsev Vyacheslav 212 group
int main() {
    Interface interface;
    interface.run();
    return 0;
}
