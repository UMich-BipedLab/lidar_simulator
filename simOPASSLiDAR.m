%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function [diodes_array, diodes_array_endpoints, diodes_array_endpoints_noisy, noisy_model] = simOPASSLiDAR(LiDAR_opts, object)
    diodes_array = genLiDARArray(LiDAR_opts);
    diodes_array_endpoints = zeros(4, LiDAR_opts.properties.diode_array^2);
    diodes_array_endpoints_noisy = zeros(4, LiDAR_opts.properties.diode_array^2);

    h_resolution = 2*LiDAR_opts.properties.h_coverage / LiDAR_opts.properties.diode_array;
    v_resolution = 2*LiDAR_opts.properties.v_coverage / LiDAR_opts.properties.diode_array;

    % parfor prepare
    num_points = (LiDAR_opts.properties.diode_array)^2;
    segmentation_type(num_points).angle.row = [];
    segmentation_type(num_points).angle.column = [];
    segmentation_type(num_points).angle.point = [];
    segmentation_type(num_points).angle.azimuth = [];
    segmentation_type(num_points).angle.elevation = [];
    segmentation_type(num_points).geom.objects = [];
    [normal, centroid] = computePlane(object);
    
    parfor point = 1:num_points
%     for point = 1:num_points
        [row, column] = findArrayToMatrixIndex(point, LiDAR_opts.properties.diode_array);
        % column should decide azimuth (z)
        % row should decide elevation  (y)
        azimuth  = LiDAR_opts.properties.h_coverage - h_resolution * column;
        elevation = LiDAR_opts.properties.v_coverage - v_resolution * row;
        start_point = diodes_array.diode(:, point).point;
        end_piont = measureSolidStateEndPoints(start_point, diodes_array.diode(:, point).T, [0 elevation azimuth], LiDAR_opts.pose.range);
        [diodes_array_endpoints(:, point), inside_polygon] = checkInsidePolygonGivenPlaneAndTwoPoints(object, start_point, end_piont);
        segmentation_type(point).geom.objects = object;
        segmentation_type(point).angle.normals = normal;
        segmentation_type(point).angle.centroid = centroid;
        segmentation_type(point).angle.row = row;
        segmentation_type(point).angle.column = column;
        segmentation_type(point).angle.point = diodes_array_endpoints(:, point);
        segmentation_type(point).angle.azimuth = azimuth;
        segmentation_type(point).angle.elevation = elevation;
        noisy_model = genLiDARNoiseModel(segmentation_type(point), LiDAR_opts);
        %%% XXX Noise now only support when the array placed in x-axis
        diodes_array_endpoints_noisy(:, point) = [addTwoTypesOfNoise(diodes_array_endpoints(:, point), [], noisy_model);1];
    end
end