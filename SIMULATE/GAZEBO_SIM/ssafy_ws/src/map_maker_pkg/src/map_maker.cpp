#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>

// 파라미터 설정 (언제든 수정 가능)
// 저장 경로 및 파일명 설정 (언제든 수정 가능)
const std::string SAVE_DIRECTORY = "map/"; // 저장할 디렉토리
const std::string FILE_PREFIX = "path_"; // 파일 이름 접두사
const std::string FILE_EXTENSION = ".csv"; // 확장자

// 점 간격 설정 (10cm = 0.1m)
constexpr double STEP_SIZE = 0.1;

struct Point {
    double x, y;
};



// 두 점 사이의 거리 계산 함수
double calculateDistance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// 두 점 사이에 일정 간격으로 점 생성 함수
std::vector<Point> generatePath(const std::vector<Point>& input_points) {
    std::vector<Point> path;

    for (size_t i = 0; i < input_points.size() - 1; ++i) {
        Point start = input_points[i];
        Point end = input_points[i + 1];

        double dist = calculateDistance(start, end);
        int num_points = std::max(1, static_cast<int>(dist / STEP_SIZE));

        for (int j = 0; j <= num_points; ++j) {
            double t = static_cast<double>(j) / num_points;
            Point interpolated;
            interpolated.x = start.x + t * (end.x - start.x);
            interpolated.y = start.y + t * (end.y - start.y);
            path.push_back(interpolated);
        }
    }
    return path;
}

// 현재 날짜 및 시간을 문자열로 변환하는 함수
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return oss.str();
}

// 경로 데이터를 CSV 파일로 저장
void savePathToCSV(const std::vector<Point>& path) {
    std::string timestamp = getCurrentTimestamp();
    std::string filename = SAVE_DIRECTORY + FILE_PREFIX + timestamp + FILE_EXTENSION;

    // 폴더가 존재하지 않으면 생성
    std::filesystem::create_directories(SAVE_DIRECTORY);

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
        return;
    }

    file << "x,y\n";  // CSV 헤더 추가
    for (const auto& p : path) {
        file << p.x << "," << p.y << "\n";
    }

    file.close();
    std::cout << "경로가 성공적으로 저장되었습니다: " << filename << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("map_maker_node");

    std::vector<Point> input_points;
    int num_points;
    
    // 점 개수 입력
    while(1){
        std::cout << "점 개수를 입력하세요: ";
        std::cin >> num_points;
        if(num_points < 2) {
            std::cout << "점의 개수는 2개 이상이어야 합니다.\n";
        } else {
            break;
        }
    }

    std::cout << "각 점의 좌표를 입력하세요 (x y):\n";
    for (int i = 0; i < num_points; ++i) {
        std::cout << "점 " << i + 1 << ": ";
        Point p;
        std::cin >> p.x >> p.y;
        input_points.push_back(p);
    }

    auto path = generatePath(input_points);
    savePathToCSV(path);

    rclcpp::shutdown();
    return 0;
}
