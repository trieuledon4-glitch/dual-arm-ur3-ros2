// terminal_control_dual.cpp (ban sua v3 - on dinh)
// Dieu khien 2 tay UR3e+RG2 (arm1_, arm2_) chuyen vat theo vong A -> B -> C -> A
//
// Thay doi so voi ban truoc:
//  1. SUA LOI "vat khong nam trong ngon kep": do sau ha xuong duoc TINH tu toa do vat
//     (khong con gan cung -0.09). TCP ha toi giua than vat, co in z de kiem chung.
//  2. Chi tat kiem tra va cham o 2 buoc can thiet: ha xuong gap, va nhac len sau khi tha
//     (luc do ngon kep dang bao quanh vat nen planner se chan).
//  3. Huong gap CO DINH theo goc cua tung tay (vat tru doi xung nen khong can xoay co tay)
//     -> co tay khong quay lung tung, chuyen dong don gian va lap lai duoc.
//  4. Moi lenh di chuyen thu lai toi da 3 lan neu lap ke hoach that bai.
//  5. Cho ngam dong/mo xong moi attach/tiep tuc.
//  6. Van giu: lui ve home sau place, kiem tra cu phap, lenh clear.

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <chrono>
#include <cmath>

using MGI = moveit::planning_interface::MoveGroupInterface;
using PSI = moveit::planning_interface::PlanningSceneInterface;

// ============================================================================
// THAM SO CAN CHINH (chinh o day, khong can sua logic)
// ============================================================================
constexpr double kApproachHeight = 0.16;        // TCP o tren tam vat bao nhieu khi tiep can
constexpr double kGraspHeightAboveCenter = 0.0; // TCP o tren tam vat bao nhieu khi kep (0 = ngang tam)
constexpr double kMinTcpZ = 0.02;               // chan duoi an toan cua TCP (tranh cham ban)
constexpr double kLiftHeight = 0.10;            // do nhac len sau khi kep / sau khi tha
constexpr int    kPlanRetries = 3;
constexpr int    kGripperWaitMs = 600;          // cho ngam dong/mo xong

// ============================================================================
// KHOI 1: DINH NGHIA 2 TAY MAY
// ============================================================================
struct ArmHandle {
    std::string label;
    std::string arm_group;
    std::string gripper_group;
    std::string tcp_frame;
    std::string open_state;
    std::string closed_state;
    std::shared_ptr<MGI> arm;
    std::shared_ptr<MGI> gripper;
    std::string held_object;
    double held_tcp_offset_z = 0.0;
    double base_yaw = 0.0;               // huong gap co dinh (rad): tay 1 = 0, tay 2 = pi
    double base_x = 0.0;                 // vi tri goc tay trong world (dung cho huong du phong)
    double base_y = 0.0;
    std::vector<double> home_joints;     // tu the nghi (luc khoi dong)
};

struct ObjectData { std::string id; double x, y, z; };
std::map<std::string, ObjectData> spawned_objects;

// ============================================================================
// KHOI 2: CAC HAM DUNG CHUNG
// ============================================================================
// Huong gap CO DINH: ngam huong xuong (roll = pi), yaw co dinh theo tung tay.
geometry_msgs::msg::Quaternion graspOrientation(const ArmHandle &h) {
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, h.base_yaw);
    return tf2::toMsg(q);
}

// Huong DU PHONG: ngam huong xuong, xoay co tay quay ve phia vat (tang tam voi)
geometry_msgs::msg::Quaternion radialOrientation(const ArmHandle &h, double x, double y) {
    double yaw = std::atan2(y - h.base_y, x - h.base_x);
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, yaw);
    return tf2::toMsg(q);
}

// Lap ke hoach + thuc thi, thu lai toi da `retries` lan
bool planAndExecute(ArmHandle &h, int retries = kPlanRetries) {
    for (int i = 1; i <= retries; ++i) {
        h.arm->setStartStateToCurrentState();
        MGI::Plan plan;
        if (h.arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return (h.arm->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        std::cout << "   (" << h.label << ": lap ke hoach that bai, lan " << i << "/" << retries << ")\n";
    }
    return false;
}

bool moveCartesianZ(ArmHandle &h, double delta_z, double min_fraction = 0.95, bool avoid_collisions = true) {
    h.arm->setStartStateToCurrentState();
    geometry_msgs::msg::Pose start_pose = h.arm->getCurrentPose(h.tcp_frame).pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.z += delta_z;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = h.arm->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, avoid_collisions, nullptr);
    if (fraction >= min_fraction) {
        MGI::Plan plan;
        plan.trajectory_ = trajectory;
        return (h.arm->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    std::cout << "   (" << h.label << ": Cartesian chi di duoc " << fraction * 100.0 << "%)\n";
    return false;
}

bool moveToPose(ArmHandle &h, const geometry_msgs::msg::Pose &target_pose, int retries = kPlanRetries) {
    h.arm->setPoseTarget(target_pose, h.tcp_frame);
    bool ok = planAndExecute(h, retries);
    h.arm->clearPoseTargets();
    return ok;
}

// Di toi (x, y, z): thu huong gap co dinh truoc (don gian, on dinh);
// neu khong toi duoc thi thu huong quay ve phia vat (voi xa hon).
bool moveToPoseFlex(ArmHandle &h, double x, double y, double z) {
    geometry_msgs::msg::Pose p;
    p.position.x = x; p.position.y = y; p.position.z = z;
    p.orientation = graspOrientation(h);
    if (moveToPose(h, p, 2)) return true;
    std::cout << "   (" << h.label << ": thu huong gap quay ve phia vat de tang tam voi)\n";
    p.orientation = radialOrientation(h, x, y);
    return moveToPose(h, p, 2);
}

// Dua tay ve tu the nghi luc khoi dong (de nhuong cho tay kia)
bool retreatHome(ArmHandle &h) {
    if (h.home_joints.empty()) return false;
    std::cout << "-> " << h.label << ": lui ve tu the nghi...\n";
    h.arm->setJointValueTarget(h.home_joints);
    bool ok = planAndExecute(h);
    if (!ok) std::cout << "   (" << h.label << ": khong ve duoc tu the nghi)\n";
    return ok;
}

bool setGripper(ArmHandle &h, bool open) {
    h.gripper->setNamedTarget(open ? h.open_state : h.closed_state);
    bool ok = (h.gripper->move() == moveit::core::MoveItErrorCode::SUCCESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(kGripperWaitMs));
    if (!ok) std::cout << "   (" << h.label << ": " << (open ? "mo" : "dong") << " ngam THAT BAI)\n";
    return ok;
}

bool pickWith(ArmHandle &h, const std::string &name) {
    if (!h.held_object.empty()) {
        std::cout << "==> " << h.label << " dang kep [" << h.held_object << "], phai place truoc!\n";
        return false;
    }
    auto it = spawned_objects.find(name);
    if (it == spawned_objects.end()) {
        std::cout << "==> Khong tim thay vat [" << name << "]. Go 'list' de xem ten.\n";
        return false;
    }
    auto obj = it->second;

    std::cout << "-> " << h.label << " Buoc 1: Mo ngam...\n";
    if (!setGripper(h, true)) return false;

    std::cout << "-> " << h.label << " Buoc 2: Tiep can [" << name << "]...\n";
    if (!moveToPoseFlex(h, obj.x, obj.y, obj.z + kApproachHeight)) {
        std::cout << "==> " << h.label << ": khong toi duoc vi tri tiep can (ngoai tam voi?).\n";
        return false;
    }

    // Buoc 3: ha thang dung toi do cao kep, TINH tu z hien tai cua TCP
    std::cout << "-> " << h.label << " Buoc 3: Ha xuong gap...\n";
    double tcp_z_now = h.arm->getCurrentPose(h.tcp_frame).pose.position.z;
    double tcp_z_grasp = std::max(obj.z + kGraspHeightAboveCenter, kMinTcpZ);
    double delta = tcp_z_grasp - tcp_z_now;
    std::cout << "   (TCP z hien tai = " << tcp_z_now << ", muc tieu = " << tcp_z_grasp
              << ", ha " << delta << " m)\n";
    if (delta >= -0.005) {
        std::cout << "==> " << h.label << ": do cao tiep can bat thuong, huy.\n";
        return false;
    }
    // Tat kiem tra va cham CHI o buoc nay: ngon kep se bao quanh vat nen planner se chan
    if (!moveCartesianZ(h, delta, 0.95, false)) {
        std::cout << "==> " << h.label << ": ha xuong that bai.\n";
        return false;
    }
    double tcp_z_after = h.arm->getCurrentPose(h.tcp_frame).pose.position.z;
    std::cout << "   (Sau khi ha: TCP z = " << tcp_z_after << ", tam vat z = " << obj.z << ")\n";

    h.held_tcp_offset_z = tcp_z_after - obj.z;

    // Attach TRUOC khi dong ngam: vat attach voi touch_links thi ngon kep duoc phep cham vat,
    // nen MoveIt khong chan viec dong ngam quanh vat.
    std::cout << "-> " << h.label << " Buoc 4: Attach + dong ngam...\n";
    h.arm->attachObject(name, h.tcp_frame, h.gripper->getLinkNames());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!setGripper(h, false)) {
        h.arm->detachObject(name);
        std::cout << "==> " << h.label << ": dong ngam that bai, da huy attach.\n";
        return false;
    }
    h.held_object = name;
    std::cout << "==> " << h.label << " da kep [" << name << "] thanh cong!\n";
    return true;
}

bool placeWith(ArmHandle &h, double target_x, double target_y) {
    if (h.held_object.empty()) {
        std::cout << "==> " << h.label << " chua kep vat nao.\n";
        return false;
    }
    auto it = spawned_objects.find(h.held_object);
    if (it == spawned_objects.end()) {
        std::cout << "==> Mat thong tin vat [" << h.held_object << "].\n";
        h.held_object.clear();
        return false;
    }
    double obj_rest_z = it->second.z;

    std::cout << "-> " << h.label << " Buoc 1: Nhac len...\n";
    if (!moveCartesianZ(h, kLiftHeight, 0.8)) {
        std::cout << "==> " << h.label << ": nhac len that bai.\n";
        return false;
    }

    double tcp_z_place = obj_rest_z + h.held_tcp_offset_z + 0.01;
    double tcp_z_hover = tcp_z_place + kLiftHeight;

    std::cout << "-> " << h.label << " Buoc 2: Bay toi tren diem dat (" << target_x << ", " << target_y << ")...\n";
    if (!moveToPoseFlex(h, target_x, target_y, tcp_z_hover)) {
        std::cout << "==> " << h.label << ": khong toi duoc vi tri dat (ngoai tam voi?).\n";
        return false;
    }

    std::cout << "-> " << h.label << " Buoc 3: Ha xuong dat...\n";
    if (!moveCartesianZ(h, tcp_z_place - tcp_z_hover)) {
        geometry_msgs::msg::Pose p_place = h.arm->getCurrentPose(h.tcp_frame).pose;
        p_place.position.z = tcp_z_place;
        if (!moveToPose(h, p_place)) {
            std::cout << "==> " << h.label << ": ha xuong that bai (vat van dang kep)!\n";
            return false;
        }
    }

    std::cout << "-> " << h.label << " Buoc 4: Mo ngam + detach...\n";
    if (!setGripper(h, true)) {
        std::cout << "==> " << h.label << ": mo ngam that bai (vat van dang kep)!\n";
        return false;
    }
    h.arm->detachObject(h.held_object);
    spawned_objects[h.held_object] = {h.held_object, target_x, target_y, obj_rest_z};
    h.held_object.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    // Ngon kep dang bao quanh vat -> phai tat kiem tra va cham khi rut len
    moveCartesianZ(h, kLiftHeight, 0.8, false);

    std::cout << "-> " << h.label << " Buoc 5: Lui ra khoi vung lam viec...\n";
    retreatHome(h);

    std::cout << "==> " << h.label << " da dat vat thanh cong!\n";
    return true;
}

// ============================================================================
// KHOI 3: QUAN LY VAT THE
// ============================================================================
bool spawnTable(PSI &psi) {
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "world";
    table.id = "table";
    shape_msgs::msg::SolidPrimitive s;
    s.type = s.BOX;
    s.dimensions = {1.6, 1.0, 0.1};
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.x = 0.25; p.position.y = 0.0; p.position.z = -0.06;
    table.primitives.push_back(s);
    table.primitive_poses.push_back(p);
    table.operation = table.ADD;

    moveit_msgs::msg::ObjectColor color;
    color.id = "table";
    color.color.r = 0.6f; color.color.g = 0.45f; color.color.b = 0.3f; color.color.a = 1.0f;

    // move_group co the chua san sang ngay luc khoi dong -> thu lai va kiem tra bang cach doc lai
    for (int i = 0; i < 10; ++i) {
        psi.applyCollisionObjects({table}, {color});
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto known = psi.getKnownObjectNames();
        if (std::find(known.begin(), known.end(), "table") != known.end()) {
            std::cout << "==> Da tao ban.\n";
            return true;
        }
    }
    std::cout << "==> CANH BAO: khong tao duoc ban (move_group chua san sang?). Go 'list' de kiem tra.\n";
    return false;
}

void spawnCylinder(PSI &psi, const std::string &name, double x, double y, double z = 0.04) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "world";
    object.id = name;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions = {0.08, 0.02};   // {cao, ban kinh}
    geometry_msgs::msg::Pose pose;
    pose.position.x = x; pose.position.y = y; pose.position.z = z;
    pose.orientation.w = 1.0;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;
    psi.applyCollisionObject(object);
    spawned_objects[name] = {name, x, y, z};
    std::cout << "==> Da tao [" << name << "] tai (" << x << ", " << y << ", " << z << ")\n";
}

void listAllObjects(PSI &psi) {
    auto known = psi.getKnownObjectNames();
    std::cout << "==> Danh sach vat (" << known.size() << "):\n";
    for (auto &id : known) std::cout << "    - " << id << (id == "table" ? "  (ban)" : "") << "\n";
}

// clear: xoa vat. only rong = xoa tat ca (tru ban). Tu nha vat neu dang duoc kep.
void clearObjects(PSI &psi, ArmHandle &a1, ArmHandle &a2, const std::string &only) {
    std::vector<std::string> to_remove;

    for (ArmHandle *a : {&a1, &a2}) {
        if (!a->held_object.empty() && (only.empty() || only == a->held_object)) {
            a->arm->detachObject(a->held_object);
            a->held_object.clear();
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    if (only.empty()) {
        for (auto &id : psi.getKnownObjectNames()) {
            if (id != "table") to_remove.push_back(id);
        }
        for (auto &kv : psi.getAttachedObjects()) {
            if (std::find(to_remove.begin(), to_remove.end(), kv.first) == to_remove.end())
                to_remove.push_back(kv.first);
        }
    } else {
        to_remove.push_back(only);
    }

    if (to_remove.empty()) {
        std::cout << "==> Khong co vat nao de xoa.\n";
        return;
    }
    psi.removeCollisionObjects(to_remove);
    for (auto &id : to_remove) spawned_objects.erase(id);
    std::cout << "==> Da xoa " << to_remove.size() << " vat.\n";
}

// ============================================================================
// MAIN
// ============================================================================
void configureArm(ArmHandle &a, rclcpp::Node::SharedPtr node) {
    a.arm = std::make_shared<MGI>(node, a.arm_group);
    a.gripper = std::make_shared<MGI>(node, a.gripper_group);
    a.arm->setEndEffectorLink(a.tcp_frame);
    a.arm->setPlanningTime(5.0);
    a.arm->setNumPlanningAttempts(5);
    a.arm->setMaxVelocityScalingFactor(0.2);       // cham -> em, it rung
    a.arm->setMaxAccelerationScalingFactor(0.2);
    a.arm->setGoalPositionTolerance(0.003);
    a.arm->setGoalOrientationTolerance(0.02);
    a.arm->setGoalJointTolerance(0.005);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "terminal_control_dual",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ---- Tay 1 ----
    ArmHandle arm1;
    arm1.label = "Tay 1";
    arm1.arm_group = "arm1_ur_onrobot_manipulator";
    arm1.gripper_group = "arm1_ur_onrobot_gripper";
    arm1.tcp_frame = "arm1_gripper_tcp";
    arm1.open_state = "open";
    arm1.closed_state = "closed";
    arm1.base_yaw = 0.0;
    arm1.base_x = 0.0; arm1.base_y = 0.0;
    configureArm(arm1, node);

    // ---- Tay 2 (dat o x=0.7, xoay 180 do - khop file xacro) ----
    ArmHandle arm2;
    arm2.label = "Tay 2";
    arm2.arm_group = "arm2_ur_onrobot_manipulator";
    arm2.gripper_group = "arm2_ur_onrobot_gripper";
    arm2.tcp_frame = "arm2_gripper_tcp";
    arm2.open_state = "open2";
    arm2.closed_state = "closed2";
    arm2.base_yaw = M_PI;
    arm2.base_x = 0.7; arm2.base_y = 0.0;   // neu doi khoang cach trong xacro thi doi so nay
    configureArm(arm2, node);

    arm1.home_joints = arm1.arm->getCurrentJointValues();
    arm2.home_joints = arm2.arm->getCurrentJointValues();

    PSI psi;
    spawnTable(psi);   // in ket qua ra man hinh

    std::cout << "\n==============================================================\n";
    std::cout << "   DIEU KHIEN 2 TAY MAY - CHUYEN VAT A -> B -> C -> A\n";
    std::cout << "==============================================================\n";
    std::cout << " - spawn <x> <y> name=<id>\n";
    std::cout << " - pick1 <id>   | place1 <x> <y>   (dieu khien Tay 1)\n";
    std::cout << " - pick2 <id>   | place2 <x> <y>   (dieu khien Tay 2)\n";
    std::cout << " - relay <id> <Ax> <Ay> <Bx> <By> <Cx> <Cy>\n";
    std::cout << "     (chay tu dong ca chuoi: Tay1 A->B, Tay2 B->C, Tay1 C->A)\n";
    std::cout << " - clear | clear <id>   (xoa tat ca vat / xoa 1 vat)\n";
    std::cout << " - home1 | home2        (dua tay ve tu the nghi)\n";
    std::cout << " - list | exit\n";
    std::cout << "==============================================================\n\n";

    std::string line;
    while (rclcpp::ok()) {
        std::cout << "\n[LENH] > ";
        if (!std::getline(std::cin, line)) break;
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cmd; ss >> cmd;

        if (cmd == "exit" || cmd == "quit") break;

        if (cmd == "spawn") {
            std::string tok, name; std::vector<double> nums;
            bool ok = true;
            while (ss >> tok) {
                if (tok.rfind("name=", 0) == 0) name = tok.substr(5);
                else {
                    try { nums.push_back(std::stod(tok)); }
                    catch (...) { ok = false; }
                }
            }
            if (!ok) { std::cout << "==> Cu phap: spawn <x> <y> name=<id>\n"; continue; }
            double x = nums.size() > 0 ? nums[0] : 0.15;
            double y = nums.size() > 1 ? nums[1] : 0.20;
            if (name.empty()) name = "obj_" + std::to_string(spawned_objects.size() + 1);
            spawnCylinder(psi, name, x, y);
            continue;
        }
        if (cmd == "list") { listAllObjects(psi); continue; }

        if (cmd == "clear") {
            std::string n; ss >> n;
            clearObjects(psi, arm1, arm2, n);
            continue;
        }

        if (cmd == "home1") { retreatHome(arm1); continue; }
        if (cmd == "home2") { retreatHome(arm2); continue; }

        if (cmd == "pick1" || cmd == "pick2") {
            std::string n; ss >> n;
            if (n.empty()) { std::cout << "==> Cu phap: " << cmd << " <id>\n"; continue; }
            pickWith(cmd == "pick1" ? arm1 : arm2, n);
            continue;
        }
        if (cmd == "place1" || cmd == "place2") {
            double x, y;
            if (!(ss >> x >> y)) { std::cout << "==> Cu phap: " << cmd << " <x> <y>  (toa do, khong phai ten vat)\n"; continue; }
            placeWith(cmd == "place1" ? arm1 : arm2, x, y);
            continue;
        }

        // ---------------- RELAY ----------------
        if (cmd == "relay") {
            std::string name; double Ax, Ay, Bx, By, Cx, Cy;
            if (!(ss >> name >> Ax >> Ay >> Bx >> By >> Cx >> Cy)) {
                std::cout << "==> Cu phap: relay <id> <Ax> <Ay> <Bx> <By> <Cx> <Cy>\n";
                continue;
            }

            std::cout << "\n===== BAT DAU RELAY: " << name << " =====\n";

            std::cout << "\n--- Chang 1/3: Tay 1 gap tai A, dat tai B ---\n";
            if (!pickWith(arm1, name))    { std::cout << "==> RELAY DUNG o Chang 1 (pick).\n"; continue; }
            if (!placeWith(arm1, Bx, By)) { std::cout << "==> RELAY DUNG o Chang 1 (place).\n"; continue; }

            std::cout << "\n--- Chang 2/3: Tay 2 gap tai B, dat tai C ---\n";
            if (!pickWith(arm2, name))    { std::cout << "==> RELAY DUNG o Chang 2 (pick).\n"; continue; }
            if (!placeWith(arm2, Cx, Cy)) { std::cout << "==> RELAY DUNG o Chang 2 (place).\n"; continue; }

            std::cout << "\n--- Chang 3/3: Tay 1 gap tai C, dat tai A ---\n";
            if (!pickWith(arm1, name))    { std::cout << "==> RELAY DUNG o Chang 3 (pick).\n"; continue; }
            if (!placeWith(arm1, Ax, Ay)) { std::cout << "==> RELAY DUNG o Chang 3 (place).\n"; continue; }

            std::cout << "\n===== RELAY HOAN TAT! Vat da ve lai diem A. =====\n";
            continue;
        }

        std::cout << "==> Lenh khong hop le: " << cmd << "\n";
    }

    rclcpp::shutdown();
    if (spinner.joinable()) spinner.join();
    return 0;
}
