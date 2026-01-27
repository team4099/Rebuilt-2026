package com.team4099.robot2026.util

import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import kotlin.math.pow
import kotlin.math.sqrt
import org.tribuo.Feature
import org.tribuo.Model
import org.tribuo.MutableDataset
import org.tribuo.classification.LabelFactory
import org.tribuo.clustering.ClusterID
import org.tribuo.clustering.ClusteringFactory
import org.tribuo.clustering.hdbscan.HdbscanTrainer
import org.tribuo.impl.ArrayExample
import org.tribuo.math.distance.L2Distance
import org.tribuo.math.neighbour.NeighboursQueryFactoryType
import org.tribuo.provenance.SimpleDataSourceProvenance

/**
 * Clusters individual Translation2d object detections using HDBSCAN, scores clusters based on
 * compactness and distance to the robot, and returns the Pose2d centroid of the highest scoring
 * cluster.
 *
 * Intended for scoring gamepiece detections from vision.
 *
 * @author Aryan Singh
 */
class ClusterScore {
  companion object {

    data class ClusterScoreData(
        val clusterId: Int,
        val score: Double,
        val spread: Double,
        val distance: Double
    )

    /* ---------------- Geometry helpers ---------------- */

    private fun distance(a: Translation2d, b: Translation2d): Double = a.getDistance(b)

    private fun centroid(points: List<Translation2d>): Pose2d {
      val n = points.size.toDouble()
      val x = points.sumOf { it.x } / n
      val y = points.sumOf { it.y } / n
      return Pose2d(x, y, Rotation2d())
    }

    private fun clusterSpread(points: List<Translation2d>): Double {
      val c = centroid(points).translation
      val meanSq = points.sumOf { distance(it, c).pow(2) } / points.size
      return sqrt(meanSq)
    }

    /* ---------------- Tribuo helpers ---------------- */

    private fun createDataset(points: List<Translation2d>): MutableDataset<ClusterID> {

      val factory = ClusteringFactory()
      val dataset =
          MutableDataset(
              SimpleDataSourceProvenance("Ohio Aurablud Translation2d Dataset", LabelFactory()),
              factory)

      for (p in points) {
        val example = ArrayExample(ClusteringFactory.UNASSIGNED_CLUSTER_ID)
        example.add(Feature("x", p.x))
        example.add(Feature("y", p.y))
        dataset.add(example)
      }

      return dataset
    }

    private fun collectClusters(
        model: Model<ClusterID>,
        points: List<Translation2d>
    ): Map<Int, List<Translation2d>> {

      val clusters = mutableMapOf<Int, MutableList<Translation2d>>()

      for (p in points) {
        val example = ArrayExample(ClusteringFactory.UNASSIGNED_CLUSTER_ID)
        example.add(Feature("x", p.x))
        example.add(Feature("y", p.y))

        val clusterId = model.predict(example).output.id
        clusters.computeIfAbsent(clusterId) { mutableListOf() }.add(p)
      }

      return clusters
    }

    private fun scoreClusters(
        clusters: Map<Int, List<Translation2d>>,
        robotPose: Pose2d,
        compactnessWeight: Double,
        distanceWeight: Double,
        epsilon: Double = 1e-6
    ): List<ClusterScoreData> {

      val robotTranslation = robotPose.translation

      return clusters
          .map { (id, points) ->
            val cPose = centroid(points)
            val spread = clusterSpread(points)
            val distToRobot = distance(cPose.translation, robotTranslation)

            val score =
                compactnessWeight / (spread + epsilon) + distanceWeight / (distToRobot + epsilon)

            ClusterScoreData(id, score, spread, distToRobot)
          }
          .sortedByDescending { it.score }
    }

    /* ---------------- Public API ---------------- */

    fun calculateClusterScores(robotPose: Pose2d, points: List<Translation3d>): Pose2d {

      require(points.isNotEmpty()) { "Point list must not be empty" }

      val dataset = createDataset(points.map { it.toTranslation2d() })

      val trainer =
          HdbscanTrainer(
              3,
              L2Distance(),
              3,
              Runtime.getRuntime().availableProcessors(),
              NeighboursQueryFactoryType.BRUTE_FORCE)

      val model = trainer.train(dataset)

      val clusters =
          collectClusters(model, points.map { it.toTranslation2d() }).filterValues {
            it.size >= Constants.ClusterScore.MIN_SCORE_CLUSTER_SIZE
          }

      // Fallback: all noise
      if (clusters.isEmpty()) {
        return centroid(points.map { it.toTranslation2d() })
      }

      val scores =
          scoreClusters(
              clusters,
              robotPose,
              Constants.ClusterScore.COMPACTNESS_WEIGHT,
              Constants.ClusterScore.DISTANCE_WEIGHT)

      val bestClusterId = scores.first().clusterId
      return centroid(clusters.getValue(bestClusterId))
    }
  }
}
