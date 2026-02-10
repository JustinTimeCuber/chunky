/* Copyright (c) 2021 Chunky contributors
 *
 * This file is part of Chunky.
 *
 * Chunky is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chunky is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with Chunky.  If not, see <http://www.gnu.org/licenses/>.
 */
package se.llbit.chunky.renderer;

import se.llbit.chunky.renderer.scene.Camera;
import se.llbit.chunky.renderer.scene.PathTracer;
import se.llbit.chunky.renderer.scene.RayTracer;
import se.llbit.chunky.renderer.scene.Scene;

public class PathTracingRenderer extends TileBasedRenderer {
  protected final String id;
  protected final String name;
  protected final String description;
  protected RayTracer tracer;

  public PathTracingRenderer(String id, String name, String description, RayTracer tracer) {
    this.id = id;
    this.name = name;
    this.description = description;
    this.tracer = tracer;
  }

  @Override
  public String getId() {
    return id;
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public String getDescription() {
    return description;
  }

  @Override
  public void render(DefaultRenderManager manager) throws InterruptedException {
    Scene scene = manager.bufferedScene;

    int width = scene.canvasConfig.getWidth();
    int height = scene.canvasConfig.getHeight();
    int fullWidth = scene.canvasConfig.getCropWidth();
    int fullHeight = scene.canvasConfig.getCropHeight();
    int cropX = scene.canvasConfig.getCropX();
    int cropY = scene.canvasConfig.getCropY();

    int sppPerPass = manager.context.sppPerPass();

    Camera cam = scene.camera();
    double halfWidth = fullWidth / (2.0 * fullHeight);
    double invHeight = 1.0 / fullHeight;

    double[] sampleBuffer = scene.getSampleBuffer();

    while (scene.spp < scene.getTargetSpp()) {
      int spp = scene.spp;
      int branchCount = (tracer instanceof PathTracer) ? scene.getCurrentBranchCount() : 1;
      double sinv = 1.0 / (sppPerPass * branchCount + spp);

      submitTiles(manager, (state, pixel) -> {
        int x = pixel.firstInt();
        int y = pixel.secondInt();
        int offset = 3 * (y*width + x);
        if(scene.spp >= 100 && Math.random() > 0.02 + 0.98*(scene.getVarianceBuffer()[offset + 2]/0.001)) {
          return;
        }

        double sr = 0;
        double sg = 0;
        double sb = 0;

        double vr = 0;
        double vg = 0;
        double vb = 0;

        for (int k = 0; k < sppPerPass; k++) {
          double ox = state.random.nextDouble();
          double oy = state.random.nextDouble();

          cam.calcViewRay(state.ray, state.random,
              -halfWidth + (x + ox + cropX) * invHeight,
              -0.5 + (y + oy + cropY) * invHeight);
          scene.rayTrace(tracer, state);

          sr += state.ray.color.x * branchCount;
          sg += state.ray.color.y * branchCount;
          sb += state.ray.color.z * branchCount;

          vr += (state.ray.color.x * state.ray.color.x) * branchCount;
          vg += (state.ray.color.y * state.ray.color.y) * branchCount;
          vb += (state.ray.color.z * state.ray.color.z) * branchCount;
        }

        sampleBuffer[offset] = (sampleBuffer[offset] * spp + sr) * sinv;
        sampleBuffer[offset + 1] = (sampleBuffer[offset + 1] * spp + sg) * sinv;
        sampleBuffer[offset + 2] = (sampleBuffer[offset + 2] * spp + sb) * sinv;

        if(spp == 0) {
          scene.getVarianceBuffer()[offset] = 0;
          scene.getVarianceBuffer()[offset + 1] = 0;
          scene.getVarianceBuffer()[offset + 2] = 0;
        }
        scene.getVarianceBuffer()[offset] += vr + vg + vb;
        scene.getVarianceBuffer()[offset + 1] += branchCount;
      });

      manager.pool.awaitEmpty();
      int sppOld = scene.spp;
      scene.spp += sppPerPass * branchCount;
      // Compute blurred variance map
      if(scene.spp/100 > sppOld/100) {
        long ts = System.nanoTime();
        double[] computedVariance = scene.getComputedVariance();
        double[] integral = new double[width * height];
        // Create integral table of original variance data
        for (int y = 0; y < height; y++) {
          double rowSum = 0.0;
          for (int x = 0; x < width; x++) {
            // Normalize variance against (pixel brightness + 0.1)
            rowSum += computedVariance[y * width + x] / (0.1 + sampleBuffer[3*(y * width + x)] + sampleBuffer[3*(y * width + x) + 1] + sampleBuffer[3*(y * width + x) + 2]);
            double above = (y > 0) ? integral[(y - 1) * width + x] : 0.0;
            integral[y * width + x] = rowSum + above;
          }
        }
        // Use integral table to compute averages
        int r = 5;
        for (int y = 0; y < height; y++) {
          int y0 = Math.max(0, y - r);
          int y1 = Math.min(height - 1, y + r);

          for (int x = 0; x < width; x++) {
            int x0 = Math.max(0, x - r);
            int x1 = Math.min(width - 1, x + r);

            double A = (x0 > 0 && y0 > 0) ? integral[(y0 - 1) * width + (x0 - 1)] : 0.0; // Top left corner
            double B = (y0 > 0) ? integral[(y0 - 1) * width + x1] : 0.0; // Top right corner
            double C = (x0 > 0) ? integral[y1 * width + (x0 - 1)] : 0.0; // Bottom left corner
            double D = integral[y1 * width + x1]; // Bottom right corner

            double sum = (D - B) - (C - A);
            int area = (x1 - x0 + 1) * (y1 - y0 + 1);

            scene.getVarianceBuffer()[3*(y * width + x) + 2] = sum / area;
          }
        }
        System.out.println(System.nanoTime() - ts);

      }
      if (postRender.getAsBoolean()) break;
    }
  }
}
