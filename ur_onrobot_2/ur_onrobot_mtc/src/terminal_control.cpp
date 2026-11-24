#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <chrono>
#include <cmath>

const std::string ARM_GROUP = "ur_onrobot_manipulator";
const std::string GRIPPER_GROUP = "ur_onrobot_gripper";
const std::string TCP_FRAME = "gripper_tcp";

struct ObjectData {
    std::string id;
    double x, y, z;
};

std::map<std::string, ObjectData> spawned_objects;

// [SUA] Nho vat dang kep va do cao TCP so voi tam vat luc kep,
// de place dung lai dung do cao do thay vi hard-code z = 0.12.
std::string held_object;
// Vi tri mac dinh khi go "spawn" tro tron. Lenh "spawnhere" se cap nhat 2 gia tri nay.
double default_obj_x = 0.3, default_obj_y = -0.2;
double held_tcp_offset_z = 0.07;

geometry_msgs::msg::Quaternion computeGraspOrientation(double target_x, double target_y) {
    double yaw = std::atan2(target_y, target_x);
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, yaw);
    return tf2::toMsg(q);
}

// [SUA] Nguong fraction tu 0.7 -> 0.95. Truoc day chi di duoc 70% duong ha
// van chay va tra ve "thanh cong", dan den kep khi gripper chua xuong toi noi.
bool moveCartesianZ(moveit::planning_interface::MoveGroupInterface &arm, double delta_z,
                    double min_fraction = 0.95) {
    geometry_msgs::msg::Pose start_pose = arm.getCurrentPose(TCP_FRAME).pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.z += delta_z;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, trajectory, true, nullptr);
    if (fraction >= min_fraction) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return (arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    // Chan doan: tinh lai khi TAT kiem tra va cham. Neu % tang manh => bi va cham (bo qua vat can);
    // neu van y nguyen => loi IK / gioi han khop / gan diem ky di.
    moveit_msgs::msg::RobotTrajectory tmp_traj;
    double fraction_nocol = arm.computeCartesianPath(waypoints, 0.005, 0.0, tmp_traj, false, nullptr);
    std::cout << "   (Cartesian chi di duoc " << fraction * 100.0 << "%; khi TAT kiem tra va cham: "
              << fraction_nocol * 100.0 << "% => "
              << (fraction_nocol > fraction + 0.05 ? "BI VA CHAM" : "LOI IK / GIOI HAN KHOP") << ")\n";
    return false;
}

// [SUA] In ra thoi gian plan de biet buoc nao cham.
bool moveToPose(moveit::planning_interface::MoveGroupInterface &arm, const geometry_msgs::msg::Pose &target_pose) {
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose, TCP_FRAME);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto t0 = std::chrono::steady_clock::now();
    bool planned = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "   (plan mat " << dt << " s)\n";
    arm.clearPoseTargets();

    if (planned) {
        return (arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    return false;
}

void spawnTable(moveit::planning_interface::PlanningSceneInterface &psi) {
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "world";
    table.id = "table";
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions = {1.0, 1.0, 0.1};

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.06;

    table.primitives.push_back(table_primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    psi.applyCollisionObject(table);
}

void spawnCylinder(moveit::planning_interface::PlanningSceneInterface &psi,
                   const std::string &name, double x, double y, double z = 0.04) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "world";
    object.id = name;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions = {0.08, 0.02}; // cao 8cm, ban kinh 2cm

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;

    psi.applyCollisionObject(object);
    spawned_objects[name] = {name, x, y, z};
    std::cout << "==> Da tao [" << name << "] tai toa do (" << x << ", " << y << ", " << z << ")\n";
}

// [SUA] Ten mac dinh khong con dua vao size() (bi trung ten sau khi remove).
std::string nextFreeName(moveit::planning_interface::PlanningSceneInterface &psi) {
    auto known = psi.getKnownObjectNames();
    for (int i = 1;; ++i) {
        std::string n = "obj_" + std::to_string(i);
        if (spawned_objects.count(n)) continue;
        if (std::find(known.begin(), known.end(), n) != known.end()) continue;
        return n;
    }
}

bool removeObject(moveit::planning_interface::PlanningSceneInterface &psi,
                  moveit::planning_interface::MoveGroupInterface &arm,
                  const std::string &name) {
    arm.detachObject(name);
    std::vector<std::string> ids_to_remove{name};
    psi.removeCollisionObjects(ids_to_remove);
    spawned_objects.erase(name);
    if (held_object == name) held_object.clear(); // [SUA]
    std::cout << "==> Da xoa vat the [" << name << "] khoi RViz (neu no ton tai).\n";
    return true;
}

void listAllObjects(moveit::planning_interface::PlanningSceneInterface &psi) {
    auto known = psi.getKnownObjectNames();
    if (known.empty()) {
        std::cout << "==> Hien khong co vat the nao trong Planning Scene.\n";
        return;
    }
    std::cout << "==> Danh sach vat the dang co trong RViz (" << known.size() << "):\n";
    for (const auto &id : known) {
        std::cout << "    - " << id << (id == "table" ? "  (ban, KHONG nen xoa)" : "") << "\n";
    }
}

void clearAllObjects(moveit::planning_interface::PlanningSceneInterface &psi,
                     moveit::planning_interface::MoveGroupInterface &arm) {
    auto known = psi.getKnownObjectNames();
    std::vector<std::string> ids_to_remove;
    for (const auto &id : known) {
        if (id == "table") continue;
        arm.detachObject(id);
        ids_to_remove.push_back(id);
    }
    if (ids_to_remove.empty()) {
        std::cout << "==> Hien khong co vat nao (ngoai table) de xoa.\n";
        return;
    }
    psi.removeCollisionObjects(ids_to_remove);
    spawned_objects.clear();
    held_object.clear(); // [SUA]
    std::cout << "==> Da xoa toan bo " << ids_to_remove.size() << " vat khoi RViz (giu lai table).\n";
}

// ---------------- POSE TU LUU (thay cho pose trong SRDF) ----------------
std::map<std::string, std::vector<double>> saved_poses;
std::string poses_file;

void savePosesToFile() {
    std::ofstream f(poses_file);
    f << std::setprecision(10);
    for (const auto &kv : saved_poses) {
        f << kv.first;
        for (double v : kv.second) f << " " << v;
        f << "\n";
    }
}

void loadPosesFromFile() {
    std::ifstream f(poses_file);
    std::string line;
    while (std::getline(f, line)) {
        std::stringstream ls(line);
        std::string name;
        ls >> name;
        if (name.empty()) continue;
        std::vector<double> v;
        double x;
        while (ls >> x) v.push_back(x);
        if (!v.empty()) saved_poses[name] = v;
    }
}

void recordPose(moveit::planning_interface::MoveGroupInterface &arm, const std::string &name) {
    saved_poses[name] = arm.getCurrentJointValues();
    savePosesToFile();
    std::cout << "   (da luu pose '" << name << "')\n";
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "terminal_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    std::this_thread::sleep_for(std::chrono::seconds(1));

    moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper(node, GRIPPER_GROUP);
    moveit::planning_interface::PlanningSceneInterface psi;

    // [SUA] Dam bao computeCartesianPath, setPoseTarget va getCurrentPose cung dung 1 frame TCP.
    arm.setEndEffectorLink(TCP_FRAME);

    // [SUA] 10s x 15 attempts la qua nhieu; neu 5s ma khong ra thi thuong la IK/goal co van de,
    // cho them cung khong giup gi. Dung sai chat (mac dinh 1e-4) lam vung dich cuc nho -> kho tim IK.
    arm.setPlanningTime(5.0);
    arm.setNumPlanningAttempts(5);
    arm.setGoalPositionTolerance(0.002);     // 2 mm
    arm.setGoalOrientationTolerance(0.02);   // ~1 do
    // arm.setPlannerId("RRTConnectkConfigDefault"); // bo comment neu ten nay co trong ompl_planning.yaml cua ban
    arm.setMaxVelocityScalingFactor(0.3);
    arm.setMaxAccelerationScalingFactor(0.3);
    gripper.setMaxVelocityScalingFactor(0.5);

    spawnTable(psi);

    const char *home_dir = std::getenv("HOME");
    poses_file = std::string(home_dir ? home_dir : ".") + "/.terminal_control_poses.txt";
    loadPosesFromFile();

    std::cout << "\n======================================================\n";
    std::cout << "         MOVEIT 2 TERMINAL CONTROL INTERFACE          \n";
    std::cout << "======================================================\n";
    std::cout << "Cu phap lenh:\n";
    std::cout << " - spawn hinh_tru [x] [y] [name=<id>]   (name= dung o dau cung duoc)\n";
    std::cout << "   (Luu y: Chon x, y sao cho sqrt(x^2 + y^2) <= 0.38 de tranh ket IK)\n";
    std::cout << " - pick <id>\n";
    std::cout << " - place <x> <y>\n";
    std::cout << " - remove <id>\n";
    std::cout << " - clear\n";
    std::cout << " - list\n";
    std::cout << " - test_configuration | open | closed\n";
    std::cout << "\nCHE DO TUNG BUOC (giong video):\n";
    std::cout << " - spawn            (tao table + vat 'object')\n";
    std::cout << " - spawnhere        (tao vat ngay duoi tay kep, de pose 'pick' gap trung vat)\n";
    std::cout << " - savepose <ten> | poses   (luu / xem pose; pick, place tu dong duoc luu khi chay tu dong)\n";
    std::cout << " - home | up | gap_vat | pick | place   (pose dat san trong SRDF)\n";
    std::cout << " - open | closed    (mo/dong RG2)\n";
    std::cout << " - attach [id] | detach [id]\n";
    std::cout << " - q                (thoat)\n";
    std::cout << "(pick/place KHONG kem tham so = pose dat san; co tham so = tu dong ca chu trinh)\n";
    std::cout << "======================================================\n\n";

    std::string line;
    while (rclcpp::ok()) {
        std::cout << "\n[LENH] > ";
        if (!std::getline(std::cin, line)) break;
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string cmd;
        ss >> cmd;

        if (cmd == "exit" || cmd == "quit" || cmd == "q") break;

        // Co tham so di kem khong? (pick/place khong tham so = named pose, co tham so = tu dong)
        std::stringstream probe(line);
        std::string t1, t2;
        probe >> t1 >> t2;
        bool has_arg = !t2.empty();

        // ---------------- SPAWN ----------------
        // [SUA] LOI GOC cua "spawn hinh_tru name=vat1": code cu doc "name=vat1" vao bien double x,
        // doc that bai -> x = 0, stream bi failbit nen `ss >> name_arg` khong doc nua -> ten van la obj_1
        // va toa do thanh (0, -0.2). Gio doc tung token: co "name=" thi la ten, con lai la so.
        if (cmd == "spawn") {
            std::string type, tok, name;
            ss >> type;
            std::vector<double> nums;
            bool bad = false;
            while (ss >> tok) {
                if (tok.rfind("name=", 0) == 0) {
                    name = tok.substr(5);
                } else {
                    try {
                        nums.push_back(std::stod(tok));
                    } catch (...) {
                        std::cout << "==> Tham so khong hop le: [" << tok << "]\n";
                        bad = true;
                        break;
                    }
                }
            }
            if (bad) continue;

            double x = nums.size() > 0 ? nums[0] : default_obj_x;
            double y = nums.size() > 1 ? nums[1] : default_obj_y;
            if (type.empty()) {            // "spawn" tro tron: tao table + vat "object" nhu video
                spawnTable(psi);
                if (name.empty()) name = "object";
            }
            if (name.empty()) name = nextFreeName(psi);

            double radius = std::sqrt(x * x + y * y);
            if (radius > 0.38) {
                std::cout << "[CANH BAO] Toa do cach goc " << radius
                          << "m, gan bien tam voi cua UR3e, de bi loi/cham IK!\n";
            }

            spawnCylinder(psi, name, x, y);
            continue;
        }

        // SPAWNHERE: tao vat ngay duoi tay kep hien tai, de pose "pick" dat san gap trung vat.
        // Cach dung: go "pick" (tay toi vi tri gap) -> go "spawnhere" -> ghi lai toa do in ra.
        // Cac lan sau go "spawn" tro tron se tao vat dung cho do (trong cung phien chay).
        if (cmd == "spawnhere") {
            std::string name;
            ss >> name;
            if (name.empty()) name = "object";
            auto tp = arm.getCurrentPose(TCP_FRAME).pose;
            default_obj_x = tp.position.x;
            default_obj_y = tp.position.y;
            spawnCylinder(psi, name, tp.position.x, tp.position.y);
            std::cout << "==> Ghi lai: spawn hinh_tru " << tp.position.x << " " << tp.position.y
                      << " name=" << name << "  (dung lenh nay cho lan sau)\n";
            continue;
        }

        if (cmd == "savepose") {
            std::string name;
            ss >> name;
            if (name.empty()) {
                std::cout << "==> Go: savepose <ten>, vi du: savepose pick\n";
                continue;
            }
            recordPose(arm, name);
            continue;
        }
        if (cmd == "poses") {
            if (saved_poses.empty()) {
                std::cout << "==> Chua co pose nao duoc luu.\n";
            } else {
                std::cout << "==> Pose da luu:";
                for (const auto &kv : saved_poses) std::cout << " " << kv.first;
                std::cout << "\n";
            }
            continue;
        }

        if (cmd == "list") { listAllObjects(psi); continue; }

        if (cmd == "remove") {
            std::string name;
            ss >> name;
            if (name.empty()) {
                std::cout << "==> Ban can go ten vat, vi du: remove obj_1\n";
                continue;
            }
            removeObject(psi, arm, name);
            continue;
        }

        if (cmd == "clear") { clearAllObjects(psi, arm); continue; }

        // ---------------- PICK ----------------
        if (cmd == "pick" && has_arg) {
            std::string name;
            ss >> name;

            if (!held_object.empty()) {
                std::cout << "==> Dang kep [" << held_object << "], hay place truoc!\n";
                continue;
            }
            if (spawned_objects.find(name) == spawned_objects.end()) {
                std::cout << "==> Khong tim thay vat the ten [" << name << "]. Go 'list' de xem ten.\n";
                continue;
            }

            auto obj = spawned_objects[name];

            std::cout << "-> Buoc 1: Mo ngam RG2...\n";
            gripper.setNamedTarget("open");
            gripper.move();

            std::cout << "-> Buoc 2: Tiep can ben tren vat the [" << name << "]...\n";
            geometry_msgs::msg::Pose p_approach;
            p_approach.orientation = computeGraspOrientation(obj.x, obj.y);
            p_approach.position.x = obj.x;
            p_approach.position.y = obj.y;
            p_approach.position.z = obj.z + 0.16;

            if (!moveToPose(arm, p_approach)) {
                std::cout << "==> Khong the quy hoach tiep can! (Loi IK hoac vuot tam voi)\n";
                continue;
            }

            recordPose(arm, "pick_above");
            std::cout << "-> Buoc 3: Ha thang dung om sat vat the...\n";
            if (!moveCartesianZ(arm, -0.09)) {
                std::cout << "==> Ha xuong that bai do vuong va cham mat ban/vat the!\n";
                continue;
            }

            // [SUA] Ghi lai TCP cao hon tam vat bao nhieu luc kep, de place dung lai.
            recordPose(arm, "pick");
            held_tcp_offset_z = arm.getCurrentPose(TCP_FRAME).pose.position.z - obj.z;

            std::cout << "-> Buoc 4: Dong ngam va attach [" << name << "]...\n";
            gripper.setNamedTarget("closed");
            gripper.move();

            arm.attachObject(name, TCP_FRAME, gripper.getLinkNames());
            held_object = name;
            std::cout << "==> Da kep va attach [" << name << "] thanh cong!\n";
            continue;
        }

        // ---------------- PLACE ----------------
        if (cmd == "place" && has_arg) {
            if (held_object.empty()) {
                std::cout << "==> Chua kep vat nao. Hay 'pick <id>' truoc!\n";
                continue;
            }
            auto it = spawned_objects.find(held_object);
            if (it == spawned_objects.end()) {
                std::cout << "==> Mat thong tin vat [" << held_object << "], hay remove/clear roi thu lai.\n";
                held_object.clear();
                continue;
            }
            double obj_rest_z = it->second.z;

            double target_x = 0.3, target_y = -0.2;
            ss >> target_x >> target_y;

            std::cout << "-> Buoc 1: Nhac vat the len (Lift)...\n";
            // Lift chi can du cao de vat khoi cham ban: 10cm, chap nhan >= 80% (~8cm).
            if (!moveCartesianZ(arm, 0.10, 0.8)) {
                std::cout << "==> Nhac len that bai, dung place.\n";
                continue;
            }

            // [SUA] GIU NGUYEN HUONG ngam luc dang kep. Vat hinh tru doi xung nen khong can xoay yaw
            // theo vi tri dat. Code cu ep yaw = atan2(y, x) -> co tay phai xoay ~80 do + dich vi tri,
            // IK kho, vung dich hep. Neu that su can ngam huong vao tam, doi thanh computeGraspOrientation().
            geometry_msgs::msg::Quaternion keep_orient = arm.getCurrentPose(TCP_FRAME).pose.orientation;

            // [SUA] Do cao dat lay theo luc kep (+1cm), thay vi hard-code 0.12.
            double tcp_z_place = obj_rest_z + held_tcp_offset_z + 0.01;
            double tcp_z_hover = tcp_z_place + 0.10;

            std::cout << "-> Buoc 2: Chuyen toi phia tren toa do dat (" << target_x << ", " << target_y << ")...\n";
            geometry_msgs::msg::Pose p_hover;
            p_hover.orientation = keep_orient;
            p_hover.position.x = target_x;
            p_hover.position.y = target_y;
            p_hover.position.z = tcp_z_hover;

            if (!moveToPose(arm, p_hover)) {
                std::cout << "==> Quy hoach vi tri place that bai!\n";
                continue;
            }

            recordPose(arm, "place_above");
            std::cout << "-> Buoc 3: Ha thang xuong vi tri dat...\n";
            if (!moveCartesianZ(arm, tcp_z_place - tcp_z_hover)) {
                // Du phong: Cartesian that bai thi plan thang toi diem dat (quang ngan, se nhanh).
                std::cout << "   Thu plan thang toi diem dat thay cho Cartesian...\n";
                geometry_msgs::msg::Pose p_place = p_hover;
                p_place.position.z = tcp_z_place;
                if (!moveToPose(arm, p_place)) {
                    std::cout << "==> Ha xuong vi tri dat that bai! (vat van dang kep)\n";
                    continue;
                }
            }

            recordPose(arm, "place");
            std::cout << "-> Buoc 4: Mo ngam va detach...\n";
            gripper.setNamedTarget("open");
            gripper.move();
            arm.detachObject(held_object);

            // [SUA] Cap nhat vi tri moi cua vat, neu khong "pick" lan sau se di ve cho CU.
            spawned_objects[held_object] = {held_object, target_x, target_y, obj_rest_z};
            held_object.clear();

            moveCartesianZ(arm, 0.10);
            std::cout << "==> Place hoan tat thanh cong!\n";
            continue;
        }

        // ---------------- ATTACH / DETACH THU CONG ----------------
        if (cmd == "attach") {
            std::string name;
            ss >> name;
            if (name.empty()) name = (spawned_objects.size() == 1) ? spawned_objects.begin()->first : std::string("object");
            arm.attachObject(name, TCP_FRAME, gripper.getLinkNames());
            held_object = name;
            std::cout << "==> Da gan vat [" << name << "] vao tay kep (Attach success)\n";
            continue;
        }
        if (cmd == "detach") {
            std::string name;
            ss >> name;
            if (name.empty()) name = !held_object.empty() ? held_object : std::string("object");
            arm.detachObject(name);
            if (held_object == name) held_object.clear();
            // Dong bo lai vi tri vat de lenh "pick <id>" sau nay di dung cho
            auto poses = psi.getObjectPoses(std::vector<std::string>{name});
            if (poses.count(name)) {
                const auto &pp = poses[name].position;
                spawned_objects[name] = {name, pp.x, pp.y, pp.z};
            }
            std::cout << "==> Da nha vat [" << name << "] (Detach success)\n";
            continue;
        }

        if (cmd == "open" || cmd == "closed") {
            gripper.setNamedTarget(cmd);
            bool ok = (gripper.move() == moveit::core::MoveItErrorCode::SUCCESS);
            std::cout << (ok ? "==> Thuc hien thanh cong: " : "==> Thuc hien that bai: ") << cmd << "\n";
            continue;
        }

        // Pose da luu (savepose / tu dong ghi) duoc uu tien truoc pose trong SRDF
        {
            auto sp = saved_poses.find(cmd);
            if (sp != saved_poses.end()) {
                arm.setJointValueTarget(sp->second);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool ok = false;
                if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    ok = (arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                }
                std::cout << (ok ? "==> Thuc hien thanh cong: " : "==> Thuc hien that bai: ") << cmd << "\n";
                continue;
            }
        }

        if (arm.setNamedTarget(cmd)) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool ok = false;
            if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                ok = (arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            }
            std::cout << (ok ? "==> Thuc hien thanh cong: " : "==> Thuc hien that bai: ") << cmd << "\n";
            continue;
        }

        std::cout << "==> Lenh khong hop le: " << cmd << "\n";
    }

    rclcpp::shutdown();
    if (spinner.joinable()) spinner.join();
    return 0;
}
