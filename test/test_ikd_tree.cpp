// Comprehensive tests for ikd-Tree: every existing public operation plus the new
// point-lifetime (TTL) feature, including a check that TTL stays inert when disabled and a
// concurrency stress run that forces the background rebuild thread to engage (for the
// ThreadSanitizer / AddressSanitizer / UBSan builds).
//
// Self-contained: a tiny assertion harness, no gtest. Spatial-query correctness is checked
// against brute-force reference scans.
//
// Notes on faithful testing:
//  * Points are random and effectively distinct. A perfect integer grid would make many
//    points share a coordinate on a split axis, which exercises ikd-Tree's strict-'<' KD
//    descent ambiguity (a known property; real LiDAR points are never exactly coincident).
//  * ikd-Tree boxes are HALF-OPEN: a point is inside when min <= p < max.
//  * The tree must be non-empty before Add_Points (Add_Points on a never-Built tree
//    dereferences a null Root_Node — a pre-existing contract). TTL tests seed one far-away
//    sentinel via Build() so the real, TTL-stamped points can be added afterwards.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include "ikd_Tree.h"

using ikdtree::BoxPointType;
using ikdtree::KD_TREE;
using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;
using TreePtr = std::unique_ptr<KD_TREE<PointType>>;

// ----------------------------- test harness -----------------------------
static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                \
  do {                                                                  \
    ++g_checks;                                                         \
    if (!(cond)) {                                                      \
      ++g_failures;                                                     \
      std::printf("  [FAIL] %s  (%s:%d)\n", (msg), __FILE__, __LINE__); \
    }                                                                   \
  } while (0)

#define SECTION(name) std::printf("[ RUN ] %s\n", name)

static PointType P(float x, float y, float z) {
  PointType p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

static float dist2(const PointType &a, const PointType &b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
}

// Half-open, matching ikd-Tree's Box_Search / Delete_by_range semantics.
static bool in_box(const PointType &p, const BoxPointType &b) {
  return p.x >= b.vertex_min[0] && p.x < b.vertex_max[0] && p.y >= b.vertex_min[1] && p.y < b.vertex_max[1] &&
         p.z >= b.vertex_min[2] && p.z < b.vertex_max[2];
}

// n random, effectively-distinct points in [lo, hi]^3.
static PointVector rand_points(int n, unsigned seed, float lo = 0.0f, float hi = 100.0f) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(lo, hi);
  PointVector v;
  v.reserve(n);
  for (int i = 0; i < n; ++i) v.push_back(P(u(rng), u(rng), u(rng)));
  return v;
}

static TreePtr new_tree(float del = 0.3f, float bal = 0.6f, float box = 0.2f) {
  return std::make_unique<KD_TREE<PointType>>(del, bal, box);
}

// A tree primed with one far-away sentinel so Add_Points can be used (Root_Node non-null).
// The sentinel is Build()-ed (not TTL-stamped) so it never expires and never interferes with
// queries near the origin. Callers account for it as +1 valid point.
static const int SENTINEL = 1;
static TreePtr primed_tree(float del = 0.3f, float bal = 0.6f, float box = 0.2f) {
  TreePtr t = new_tree(del, bal, box);
  PointVector s{P(1e7f, 1e7f, 1e7f)};
  t->Build(s);
  return t;
}

static bool findable(KD_TREE<PointType> &tree, const PointType &q) {
  PointVector nn;
  std::vector<float> nd;
  tree.Nearest_Search(q, 1, nn, nd);
  return !nn.empty() && nd[0] < 1e-6f;
}

// ----------------------------- existing-feature tests -----------------------------

static void test_build_size_validnum() {
  SECTION("Build / size / validnum");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(200, 1);
  tree->Build(pts);
  CHECK(tree->size() == 200, "size after Build equals point count");
  CHECK(tree->validnum() == 200, "validnum equals size when nothing deleted");
}

static void test_add_points_no_downsample() {
  SECTION("Add_Points (no downsample)");
  TreePtr tree = new_tree();
  tree->Build(rand_points(50, 2));
  PointVector extra = rand_points(20, 3, 200.0f, 300.0f);
  tree->Add_Points(extra, false);
  CHECK(tree->size() == 70, "size grows by added count without downsample");
}

static void test_add_points_downsample() {
  SECTION("Add_Points (downsample collapses a voxel)");
  TreePtr tree = new_tree(0.3f, 0.6f, 1.0f);  // 1m voxel
  tree->Build(rand_points(10, 4, 0.0f, 500.0f));
  int before_valid = tree->validnum();  // size() counts lazily-deleted nodes; validnum() is the live count
  PointVector cluster;                  // many points inside a single 1m voxel near (1000,1000,1000)
  std::mt19937 rng(5);
  std::uniform_real_distribution<float> j(0.0f, 0.9f);
  for (int i = 0; i < 50; ++i) cluster.push_back(P(1000.0f + j(rng), 1000.0f + j(rng), 1000.0f + j(rng)));
  tree->Add_Points(cluster, true);
  CHECK(tree->validnum() == before_valid + 1, "downsample collapses the cluster to one live voxel point");
}

static void test_nearest_search() {
  SECTION("Nearest_Search vs brute force");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(300, 6);
  tree->Build(pts);
  std::mt19937 rng(123);
  std::uniform_real_distribution<float> u(-20.0f, 120.0f);
  for (int q = 0; q < 50; ++q) {
    PointType query = P(u(rng), u(rng), u(rng));
    PointVector nn;
    std::vector<float> nd;
    tree->Nearest_Search(query, 1, nn, nd);
    CHECK(nn.size() == 1, "nearest returns one point");
    if (nn.empty()) continue;
    float best = std::numeric_limits<float>::infinity();
    for (auto &p : pts) best = std::min(best, dist2(query, p));
    CHECK(std::fabs(best - nd[0]) < 1e-2f, "nearest distance matches brute force");
  }
}

static void test_knn_ordering() {
  SECTION("Nearest_Search k=5 vs brute force");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(300, 7);
  tree->Build(pts);
  PointType q = P(50.0f, 50.0f, 50.0f);
  PointVector nn;
  std::vector<float> nd;
  tree->Nearest_Search(q, 5, nn, nd);
  CHECK(nn.size() == 5, "knn returns k points");
  bool sorted = true;
  for (size_t i = 1; i < nd.size(); ++i)
    if (nd[i] < nd[i - 1]) sorted = false;
  CHECK(sorted, "knn distances are non-decreasing");
  std::vector<float> all;
  for (auto &p : pts) all.push_back(dist2(q, p));
  std::sort(all.begin(), all.end());
  CHECK(!nd.empty() && std::fabs(all[4] - nd[4]) < 1e-2f, "5th-nearest distance matches brute force");
}

static void test_box_search() {
  SECTION("Box_Search vs brute force (half-open)");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(400, 8);
  tree->Build(pts);
  BoxPointType box;
  box.vertex_min[0] = 20.0f;
  box.vertex_max[0] = 60.0f;
  box.vertex_min[1] = 20.0f;
  box.vertex_max[1] = 60.0f;
  box.vertex_min[2] = 20.0f;
  box.vertex_max[2] = 60.0f;
  PointVector found;
  tree->Box_Search(box, found);
  int brute = 0;
  for (auto &p : pts)
    if (in_box(p, box)) ++brute;
  CHECK((int)found.size() == brute, "box search count matches brute force");
  for (auto &p : found) CHECK(in_box(p, box), "every box-search result lies in the box");
}

static void test_radius_search() {
  SECTION("Radius_Search vs brute force");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(400, 9);
  tree->Build(pts);
  PointType c = P(50.0f, 50.0f, 50.0f);
  float radius = 25.0f;
  PointVector found;
  tree->Radius_Search(c, radius, found);
  int brute = 0;
  for (auto &p : pts)
    if (dist2(c, p) <= radius * radius) ++brute;
  CHECK(std::abs((int)found.size() - brute) <= 1, "radius search count matches brute force (boundary tol 1)");
  for (auto &p : found) CHECK(dist2(c, p) <= radius * radius + 1e-1f, "radius results within radius");
}

static void test_delete_points_and_search() {
  SECTION("Delete_Points removes points from queries");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(100, 10);
  tree->Build(pts);
  PointVector del(pts.begin(), pts.begin() + 10);
  tree->Delete_Points(del);
  CHECK(tree->validnum() == 90, "validnum drops by deleted count");
  CHECK(tree->size() <= 100, "size never grows from deletion");
  for (auto &d : del) CHECK(!findable(*tree, d), "deleted point no longer returned by search");
  // Survivors remain findable.
  CHECK(findable(*tree, pts[50]), "non-deleted point still found");
}

static void test_acquire_removed_points() {
  SECTION("acquire_removed_points returns rebuild-evicted points");
  TreePtr tree = new_tree(0.2f, 0.6f, 0.2f);  // low delete criterion -> rebuild fires
  PointVector pts = rand_points(2000, 11);    // > rebuild threshold so eviction happens
  tree->Build(pts);
  PointVector del(pts.begin(), pts.begin() + 1500);  // delete most -> forces rebuild+flatten
  tree->Delete_Points(del);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  PointVector removed;
  tree->acquire_removed_points(removed);
  CHECK(tree->validnum() == 500, "validnum reflects all deletions");
  CHECK(removed.size() > 0, "acquire returns points physically evicted by rebuild");
  CHECK(removed.size() <= 1500, "acquire never returns more than were deleted");
}

static void test_delete_box_and_readd() {
  SECTION("Delete_Point_Boxes + Add_Point_Boxes (re-add)");
  // High delete/balance criteria suppress rebuild so deletion stays lazy and Add_Point_Boxes
  // can un-delete the exact same nodes (a rebuild would physically evict them).
  TreePtr tree = new_tree(0.99f, 0.99f, 0.2f);
  PointVector pts = rand_points(400, 12);
  tree->Build(pts);
  BoxPointType box;
  box.vertex_min[0] = 10.0f;
  box.vertex_max[0] = 40.0f;
  box.vertex_min[1] = 10.0f;
  box.vertex_max[1] = 40.0f;
  box.vertex_min[2] = 10.0f;
  box.vertex_max[2] = 40.0f;
  std::vector<BoxPointType> boxes{box};
  int valid_before = tree->validnum();
  int removed = tree->Delete_Point_Boxes(boxes);
  CHECK(removed > 0, "box delete removes at least one point");
  CHECK(tree->validnum() == valid_before - removed, "validnum reflects box deletion");
  PointVector inbox;
  tree->Box_Search(box, inbox);
  CHECK(inbox.empty(), "no live points remain in the deleted box");
  // Add_Point_Boxes (ADD_BOX) reactivates a box region. Note: after Delete_Point_Boxes each
  // node's range is shrunk to exclude its deleted point, so the box no longer intersects
  // those nodes and re-add reactivates only a SUBSET -- ikd-Tree does not guarantee an exact
  // inverse here. We assert the accounting stays consistent and nothing is over-restored.
  int valid_after_del = tree->validnum();
  tree->Add_Point_Boxes(boxes);
  PointVector inbox2;
  tree->Box_Search(box, inbox2);
  CHECK((int)inbox2.size() <= removed, "re-add never restores more than were deleted");
  CHECK(tree->validnum() == valid_after_del + (int)inbox2.size(),
        "validnum stays consistent with what re-add reactivated");
  for (auto &p : inbox2) CHECK(in_box(p, box), "reactivated points lie within the box");
}

static void test_multithread_rebuild_correctness() {
  SECTION("Large add/delete triggers async rebuild; queries stay correct");
  TreePtr tree = new_tree(0.3f, 0.6f, 0.5f);
  PointVector pts = rand_points(5000, 13);  // > Multi_Thread_Rebuild_Point_Num (1500)
  tree->Build(pts);
  CHECK(tree->size() == 5000, "large build size correct");
  PointVector del(pts.begin(), pts.begin() + 3000);
  tree->Delete_Points(del);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));  // let async rebuild progress
  CHECK(tree->validnum() == 2000, "validnum correct after large delete");
  PointVector survivors(pts.begin() + 3000, pts.end());
  std::mt19937 rng(14);
  std::uniform_int_distribution<int> pick(0, (int)survivors.size() - 1);
  for (int t = 0; t < 40; ++t) CHECK(findable(*tree, survivors[pick(rng)]), "surviving point found after rebuild");
  for (int t = 0; t < 40; ++t) CHECK(!findable(*tree, del[pick(rng) % 3000]), "deleted point not found after rebuild");
}

static void test_other_point_types() {
  SECTION("PointXYZI / PointXYZINormal instantiations build + query");
  {
    auto t = std::make_unique<KD_TREE<pcl::PointXYZI>>(0.3f, 0.6f, 0.2f);
    KD_TREE<pcl::PointXYZI>::PointVector v;
    std::mt19937 rng(15);
    std::uniform_real_distribution<float> u(0, 100);
    for (int i = 0; i < 200; ++i) {
      pcl::PointXYZI p;
      p.x = u(rng);
      p.y = u(rng);
      p.z = u(rng);
      p.intensity = i;
      v.push_back(p);
    }
    t->Build(v);
    CHECK(t->size() == 200, "PointXYZI build size");
    t->Set_lifetime(100.0);
    t->Add_Points(v, false);
    CHECK(t->Remove_Expired() == 0, "PointXYZI TTL not-yet-expired");
  }
  {
    auto t = std::make_unique<KD_TREE<pcl::PointXYZINormal>>(0.3f, 0.6f, 0.2f);
    KD_TREE<pcl::PointXYZINormal>::PointVector v;
    std::mt19937 rng(16);
    std::uniform_real_distribution<float> u(0, 100);
    for (int i = 0; i < 200; ++i) {
      pcl::PointXYZINormal p;
      p.x = u(rng);
      p.y = u(rng);
      p.z = u(rng);
      p.curvature = i;
      v.push_back(p);
    }
    t->Build(v);
    CHECK(t->size() == 200, "PointXYZINormal build size");
  }
}

static void test_tree_range() {
  SECTION("tree_range covers all points");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(300, 40, 5.0f, 95.0f);
  tree->Build(pts);
  BoxPointType r = tree->tree_range();
  float minx = 1e9f, miny = 1e9f, minz = 1e9f, maxx = -1e9f, maxy = -1e9f, maxz = -1e9f;
  for (auto &p : pts) {
    minx = std::min(minx, p.x);
    maxx = std::max(maxx, p.x);
    miny = std::min(miny, p.y);
    maxy = std::max(maxy, p.y);
    minz = std::min(minz, p.z);
    maxz = std::max(maxz, p.z);
  }
  CHECK(r.vertex_min[0] <= minx + 1e-3f && r.vertex_max[0] >= maxx - 1e-3f, "tree_range x bounds enclose points");
  CHECK(r.vertex_min[1] <= miny + 1e-3f && r.vertex_max[1] >= maxy - 1e-3f, "tree_range y bounds enclose points");
  CHECK(r.vertex_min[2] <= minz + 1e-3f && r.vertex_max[2] >= maxz - 1e-3f, "tree_range z bounds enclose points");
}

static void test_param_setters_and_init() {
  SECTION("Set_*_param / set_downsample_param / InitializeKDTree keep tree usable");
  TreePtr tree = new_tree();
  tree->Set_delete_criterion_param(0.4f);
  tree->Set_balance_criterion_param(0.7f);
  tree->set_downsample_param(0.5f);
  tree->InitializeKDTree(0.4f, 0.7f, 0.5f);
  PointVector pts = rand_points(500, 41);
  tree->Build(pts);
  CHECK(tree->size() == 500, "build works after param changes");
  // A query must still be correct.
  PointType q = pts[250];
  CHECK(findable(*tree, q), "query correct after param changes");
  // Deleting enough to cross the (lowered) delete criterion must still leave a valid tree.
  PointVector del(pts.begin(), pts.begin() + 250);
  tree->Delete_Points(del);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  CHECK(tree->validnum() == 250, "validnum correct after param-driven rebuilds");
}

static void test_root_alpha() {
  SECTION("root_alpha reports sane balance/delete ratios");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(1000, 42);
  tree->Build(pts);
  PointVector d(pts.begin(), pts.begin() + 300);
  tree->Delete_Points(d);
  float a_bal = -1.0f, a_del = -1.0f;
  tree->root_alpha(a_bal, a_del);
  CHECK(a_bal >= 0.5f - 1e-3f && a_bal <= 1.0f + 1e-3f, "alpha_bal in [0.5,1]");
  CHECK(a_del >= 0.0f - 1e-3f && a_del <= 1.0f + 1e-3f, "alpha_del in [0,1]");
}

static void test_flatten_public() {
  SECTION("flatten returns exactly the live points");
  TreePtr tree = new_tree();
  PointVector pts = rand_points(400, 43);
  tree->Build(pts);
  PointVector flat;
  tree->flatten(tree->Root_Node, flat, ikdtree::NOT_RECORD);
  CHECK((int)flat.size() == tree->validnum(), "flatten count equals validnum (no deletes)");
  PointVector del(pts.begin(), pts.begin() + 100);
  tree->Delete_Points(del);
  PointVector flat2;
  tree->flatten(tree->Root_Node, flat2, ikdtree::NOT_RECORD);
  CHECK((int)flat2.size() == tree->validnum(), "flatten count equals validnum after deletes");
}

// ----------------------------- TTL feature tests -----------------------------

static void test_ttl_disabled_by_default() {
  SECTION("TTL disabled by default does not change existing behavior");
  TreePtr tree = new_tree();
  CHECK(std::isinf(tree->Get_lifetime()), "default lifetime is +inf (disabled)");
  tree->Build(rand_points(100, 20));
  PointVector more = rand_points(20, 21, 200.0f, 300.0f);
  tree->Add_Points(more, false);
  int removed = tree->Remove_Expired();
  CHECK(removed == 0, "Remove_Expired is a no-op when TTL disabled");
  CHECK(tree->size() == 120 && tree->validnum() == 120, "tree unaffected by disabled TTL");
}

static void test_ttl_not_yet_expired() {
  SECTION("TTL: fresh points are not expired");
  TreePtr tree = primed_tree();
  tree->Set_lifetime(100.0);  // long lifetime
  PointVector batch = rand_points(60, 22);
  tree->Add_Points(batch, false);
  int removed = tree->Remove_Expired();
  CHECK(removed == 0, "nothing expires within lifetime");
  CHECK(tree->validnum() == 60 + SENTINEL, "all fresh points still valid");
}

static void test_ttl_expires_old_points() {
  SECTION("TTL: old points are removed after lifetime");
  TreePtr tree = primed_tree();
  tree->Set_lifetime(0.05);  // 50 ms
  PointVector batch = rand_points(80, 23);
  tree->Add_Points(batch, false);
  CHECK(tree->validnum() == 80 + SENTINEL, "points present right after add");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  int removed = tree->Remove_Expired();
  CHECK(removed == 80, "all aged points expire");
  CHECK(tree->validnum() == SENTINEL, "only the (unstamped) sentinel remains");
  for (int i = 0; i < 20; ++i) CHECK(!findable(*tree, batch[i]), "expired point no longer searchable");
}

static void test_ttl_mixed_ages() {
  SECTION("TTL: only the aged batch expires, fresh batch survives");
  TreePtr tree = primed_tree();
  tree->Set_lifetime(0.2);  // 200 ms
  PointVector old_batch = rand_points(40, 24);
  tree->Add_Points(old_batch, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(450));  // age the first batch well past lifetime
  PointVector fresh = rand_points(30, 25, 200.0f, 300.0f);
  tree->Add_Points(fresh, false);  // stamped "now"
  int removed = tree->Remove_Expired();
  CHECK(removed == 40, "only the old batch expires");
  CHECK(tree->validnum() == 30 + SENTINEL, "fresh batch (and sentinel) survive");
  CHECK(findable(*tree, fresh[0]), "fresh point still found after old batch expiry");
  CHECK(!findable(*tree, old_batch[0]), "old point gone after expiry");
}

static void test_ttl_throttle() {
  SECTION("TTL: per-call delete cap throttles expiry");
  TreePtr tree = primed_tree();
  tree->Set_lifetime(0.05);
  tree->Set_max_expire_per_call(25);
  for (int b = 0; b < 3; ++b) {
    PointVector grp = rand_points(20, 30 + b, b * 200.0f, b * 200.0f + 100.0f);
    tree->Add_Points(grp, false);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  int first = tree->Remove_Expired();
  CHECK(first > 0 && first <= 25, "throttled call deletes whole groups up to the cap");
  int total = first;
  for (int i = 0; i < 5; ++i) total += tree->Remove_Expired();
  CHECK(total == 60, "repeated calls eventually expire everything");
  CHECK(tree->validnum() == SENTINEL, "all expired after draining");
}

static void test_ttl_no_conflict_with_manual_delete() {
  SECTION("TTL: manual delete then expiry of same points is a safe no-op");
  TreePtr tree = primed_tree();
  tree->Set_lifetime(0.05);
  PointVector batch = rand_points(50, 26);
  tree->Add_Points(batch, false);
  PointVector manual(batch.begin(), batch.begin() + 25);
  tree->Delete_Points(manual);  // delete half before they expire
  CHECK(tree->validnum() == 25 + SENTINEL, "manual delete drops half");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  int removed = tree->Remove_Expired();  // processes all 50; 25 already gone -> only 25 newly removed
  CHECK(removed == 25, "Remove_Expired returns the count ACTUALLY removed (25 already manually deleted)");
  CHECK(tree->validnum() == SENTINEL, "double-deleting already-gone points leaves a consistent tree");
}

static void test_ttl_build_clears_stale_index() {
  SECTION("TTL: Build clears stale TTL groups so rebuilt points are not expired");
  TreePtr tree = primed_tree();
  tree->Set_lifetime(1.0);

  PointVector batch = rand_points(40, 27, 10.0f, 20.0f);
  tree->Add_Points(batch, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // age only the pre-Build TTL batch

  PointVector rebuilt = batch;  // same coordinates on purpose: stale TTL would hit these
  tree->Build(rebuilt);
  CHECK(tree->validnum() == 40, "Build replaces the tree with the rebuilt batch");

  int removed = tree->Remove_Expired();

  CHECK(removed == 0, "stale TTL groups from pre-Build points must not expire rebuilt points");
  CHECK(tree->validnum() == 40, "rebuilt points remain after expiry pass");
  for (int i = 0; i < 5; ++i) CHECK(findable(*tree, rebuilt[i]), "rebuilt point still searchable");
}

static void test_ttl_build_first_scan_expires() {
  SECTION("TTL: first scan inserted by Build expires after its lifetime");
  TreePtr tree = new_tree();
  tree->Set_lifetime(0.05);

  PointVector batch = rand_points(40, 79, 50.0f, 80.0f);
  tree->Build(batch);
  CHECK(tree->validnum() == 40, "Build inserts the first scan points");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  int removed = tree->Remove_Expired();

  CHECK(removed == 40, "Build-inserted first scan is tracked by TTL and fully expires");
  CHECK(tree->validnum() == 0, "all Build-inserted points are gone after expiry");
  for (int i = 0; i < 5; ++i) CHECK(!findable(*tree, batch[i]), "expired Build point is no longer searchable");
}

static void test_ttl_expiry_drains_removed_points() {
  SECTION("TTL: expired points eventually drain through acquire_removed_points");
  TreePtr tree = new_tree(0.2f, 0.6f, 0.2f);  // low delete criterion -> rebuild/flatten records evicted points
  tree->Set_lifetime(0.03);

  PointVector batch = rand_points(2000, 80, 100.0f, 400.0f);
  tree->Build(batch);
  CHECK(tree->validnum() == 2000, "Build inserts the TTL-tracked batch before expiry");

  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  int removed = tree->Remove_Expired();
  CHECK(removed == 2000, "TTL expires the whole tracked batch");

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  PointVector drained;
  tree->acquire_removed_points(drained);

  CHECK(tree->validnum() == 0, "expired batch is fully gone from the tree");
  CHECK(drained.size() > 0, "acquire_removed_points eventually drains TTL-expired points");
  CHECK(drained.size() <= 2000, "drain never returns more points than the expired batch");
}

static void test_ttl_downsample_records_live_representative() {
  SECTION("TTL: downsample expiry tracks the live voxel representative, not raw input count");
  TreePtr tree = primed_tree(0.3f, 0.6f, 1.0f);  // 1m voxel for deterministic clustering
  tree->Set_lifetime(0.03);

  PointVector cluster{
      P(10.50f, 10.50f, 10.50f), P(10.10f, 10.10f, 10.10f), P(10.90f, 10.90f, 10.90f),
      P(10.20f, 10.80f, 10.20f), P(10.80f, 10.20f, 10.80f),
  };
  tree->Add_Points(cluster, true);

  const PointType representative = cluster[0];  // exact voxel center => chosen representative
  CHECK(tree->validnum() == SENTINEL + 1, "downsample stores exactly one live representative");
  CHECK(findable(*tree, representative), "chosen representative is searchable before expiry");

  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  int removed = tree->Remove_Expired();

  CHECK(removed == 1, "expiry deletes the single live representative, not the raw cluster size");
  CHECK(tree->validnum() == SENTINEL, "only the sentinel remains after representative expiry");
  CHECK(!findable(*tree, representative), "representative is gone after expiry");
}

static void test_build_empty_releases_static_root_owner_cleanly() {
  SECTION("Build(empty) clears the static-root owner so destruction does not revisit freed children");
  {
    TreePtr tree = new_tree();
    PointVector pts = rand_points(64, 78, 0.0f, 50.0f);
    tree->Build(pts);
    CHECK(tree->validnum() == 64, "non-empty Build populates the tree before clearing");

    PointVector empty;
    tree->Build(empty);
    CHECK(tree->size() == 0, "Build(empty) leaves the tree empty");
    CHECK(tree->validnum() == 0, "Build(empty) clears all live points");
  }  // destructor runs here; the regression was a dangling STATIC_ROOT_NODE child traversal.
  CHECK(true, "destroying after Build(empty) completes without a double free");
}

// ----------------------------- concurrency stress (sanitizers) -----------------------------

static void test_concurrency_stress_with_ttl() {
  SECTION("Concurrency stress: interleave add/delete/search/expire while rebuild runs");
  TreePtr tree = primed_tree(0.3f, 0.6f, 0.5f);
  tree->Set_lifetime(0.02);  // 20 ms, so expiry actually fires during the run
  tree->Set_max_expire_per_call(2000);
  std::mt19937 rng(2024);
  std::uniform_real_distribution<float> u(0.0f, 200.0f);
  for (int iter = 0; iter < 40; ++iter) {
    PointVector batch;
    for (int i = 0; i < 400; ++i) batch.push_back(P(u(rng), u(rng), u(rng)));
    tree->Add_Points(batch, true);  // downsample on -> exercises box-delete + add paths

    PointVector nn;
    std::vector<float> nd;
    tree->Nearest_Search(P(u(rng), u(rng), u(rng)), 8, nn, nd);  // search vs rebuild thread
    PointVector boxres;
    BoxPointType b;
    b.vertex_min[0] = 0;
    b.vertex_max[0] = 50;
    b.vertex_min[1] = 0;
    b.vertex_max[1] = 50;
    b.vertex_min[2] = 0;
    b.vertex_max[2] = 50;
    tree->Box_Search(b, boxres);

    if (iter % 3 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    tree->Remove_Expired();
  }
  tree->Remove_Expired();
  int sz = tree->size();
  int vn = tree->validnum();
  CHECK(sz >= 0 && vn >= 0 && vn <= sz, "size/validnum remain consistent after stress");
  PointVector nn;
  std::vector<float> nd;
  tree->Nearest_Search(P(100.0f, 100.0f, 100.0f), 1, nn, nd);
  // The Build()-seeded sentinel never expires, so a query must always return something --
  // a real assertion (not a tautology) that the tree stays queryable after the stress run.
  CHECK(!nn.empty(), "tree still answers a query after concurrency stress");
}

static void test_stress_add_delete_time() {
  SECTION("Stress: sustained add/delete/expire churn keeps invariants");
  TreePtr tree = primed_tree(0.3f, 0.6f, 0.3f);
  tree->Set_lifetime(0.03);  // 30 ms
  tree->Set_max_expire_per_call(5000);
  std::mt19937 rng(99);
  std::uniform_real_distribution<float> u(0.0f, 300.0f);
  long total_added = 0, total_expired = 0;
  std::vector<PointVector> recent;  // keep a few recent batches to delete manually
  for (int iter = 0; iter < 80; ++iter) {
    PointVector batch;
    for (int i = 0; i < 500; ++i) batch.push_back(P(u(rng), u(rng), u(rng)));
    tree->Add_Points(batch, true);  // downsample -> realistic churn
    total_added += batch.size();
    recent.push_back(batch);

    // Manually delete an older batch occasionally (exercise Delete_Points under churn).
    if (recent.size() > 4) {
      tree->Delete_Points(recent.front());
      recent.erase(recent.begin());
    }
    // Invariants must hold every iteration.
    int sz = tree->size(), vn = tree->validnum();
    CHECK(sz >= 0 && vn >= 0 && vn <= sz, "size/validnum invariant during churn");

    if (iter % 4 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(4));
    total_expired += tree->Remove_Expired();
  }
  // Drain remaining lifetime.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  total_expired += tree->Remove_Expired();
  std::printf("        (added=%ld expired=%ld final_valid=%d)\n", total_added, total_expired, tree->validnum());
  // NOTE: under downsample + random churn, a recorded voxel representative is often
  // superseded by a later scan, so the ACTUAL-deletion count from expiry can legitimately be
  // ~0 (the coord is no longer the live occupant). So we don't assert total_expired>0 here;
  // the invariant checks above prove no corruption under churn, and the deterministic block
  // below proves time-based expiry genuinely removes points.
  CHECK(tree->validnum() >= 0, "tree remains internally consistent after stress");
  // The tree must still answer queries correctly for a freshly added point.
  PointVector fresh;
  for (int i = 0; i < 10; ++i) fresh.push_back(P(1000.0f + i, 1000.0f, 1000.0f));
  tree->Add_Points(fresh, false);
  CHECK(findable(*tree, fresh[0]), "tree queryable after sustained stress");

  // Deterministic expiry proof (clean tree, downsample OFF, no competing deletes): a fresh
  // batch must fully expire after its lifetime and report the exact actual-deletion count.
  TreePtr clean = primed_tree();
  clean->Set_lifetime(0.03);
  PointVector fb = rand_points(200, 77, 500.0f, 800.0f);
  clean->Add_Points(fb, false);
  CHECK(clean->validnum() == 200 + SENTINEL, "stress: clean batch present before expiry");
  std::this_thread::sleep_for(std::chrono::milliseconds(90));
  int got = clean->Remove_Expired();
  CHECK(got == 200, "stress: clean batch fully expires and reports the actual removed count");
  CHECK(clean->validnum() == SENTINEL, "stress: only the sentinel remains after expiry");
}

int main() {
  std::printf("==== ikd-Tree test suite ====\n");

  // Existing features
  test_build_size_validnum();
  test_add_points_no_downsample();
  test_add_points_downsample();
  test_nearest_search();
  test_knn_ordering();
  test_box_search();
  test_radius_search();
  test_delete_points_and_search();
  test_acquire_removed_points();
  test_delete_box_and_readd();
  test_multithread_rebuild_correctness();
  test_other_point_types();
  test_tree_range();
  test_param_setters_and_init();
  test_root_alpha();
  test_flatten_public();

  // New TTL feature
  test_ttl_disabled_by_default();
  test_ttl_not_yet_expired();
  test_ttl_expires_old_points();
  test_ttl_mixed_ages();
  test_ttl_throttle();
  test_ttl_no_conflict_with_manual_delete();
  test_ttl_build_clears_stale_index();
  test_ttl_build_first_scan_expires();
  test_ttl_expiry_drains_removed_points();
  test_ttl_downsample_records_live_representative();
  test_build_empty_releases_static_root_owner_cleanly();

  // Concurrency / sanitizer stress
  test_concurrency_stress_with_ttl();
  test_stress_add_delete_time();

  std::printf("\n==== %d checks, %d failure(s) ====\n", g_checks, g_failures);
  if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
  return g_failures == 0 ? 0 : 1;
}
