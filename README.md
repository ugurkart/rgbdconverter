# How to Make an RGBD Tracker?

This is the implementation of Kart et al.'s ECCVW 2018 paper "How to Make an RGBD Tracker?" with two-level CSR-DCF integration (CSR-DCF-rgbd++ in Princeton Tracking Benchmark).

## Publication
Uğur Kart, Joni-Kristian Kämäräinen and Jiří Matas. ''How to Make an RGBD Tracker ?'' In Proceedings of the European Conference of Computer Vision Workshops (ECCVW), 2018.<br />

<b>BibTex citation:</b><br>
@InProceedings{Kart_ECCVW_2018,<br>
Title = {How to Make an RGBD Tracker?},<br>
Author = {Kart, Uğur and Kämäräinen, Joni-Kristian and Matas, Jiří},<br>
Booktitle = {ECCVW},<br>
Year = {2018}<br>
}

## Contact

Uğur Kart, e-mail: ugur.kart@tut.fi <br />

## Installation and demo
* Clone git repository: <br />
    $ git clone https://github.com/ugurkart/rgbdconverter.git
* Compile mex files running compile.m command <br />
	Set <i>opencv_include</i> and <i>opencv_libpath</i> to the correct OpenCV paths
* To replicate our results: <br />
	Download Princeton Tracking Benchmark <br />
	Please note that original PTB has many synchronization and registration errors. Therefore we use the depth images provided by Bibi et al. CVPR 2016. <br />
	Make sure Bibi et al.'s depth images are under <i>corrected_depth</i> folder in each sequence. <br />
* Use <i>run_csr.m</i> script for the visualization of the framework <br />
	Make sure all the directories of the framework are added to Matlab's path. <br />
	For the sake of demonstration, we provide one sequence from PTB, thus, you should be able to see the system running right out of the box.

## Your own tracker
* If you want to use another tracker instead of CSR-DCF, you will need to make your tracker compatible with the API we provide. We have three main functions: <br />
	<b>tracker_init</b>: Initialize the tracker with the RGB image and the BB <br />
	<b>tracker_process_frame</b>: Process the frame, output BB <br />
	<b>tracker_update</b>: Update the tracker with the new sample
	
## Project summary
We propose a generic framework to convert any arbitrary short-term RGB tracker into an RGBD tracker. The proposed framework is based on two mild requirements; the short-term tracker provides
a bounding box and its tracked object model update can be stopped and restarted. At the core of the framework is a depth augmented foreground segmentation which is formulated as an energy minimization problem and solved by graph cuts. The proposed framework offers two levels of integration. Using the first level, any RGB tracker can be converted into an RGBD tracker by allowing the framework to stop and restart the tracker according to target object visibility. The second level of integration is done if a tracker can use a mask (foreground region) in target model update. We integrate the proposed framework to one baseline tracker; Discriminative Correlation Filter (DCF), and three state-of-the-art trackers; Efficient Convolution Operators for Tracking (ECOhc, ECOgpu) and Discriminative Correlation Filter with Channel and Spatial Reliability (CSR-DCF). Comprehensive experiments on Princeton Tracking Benchmark (PTB) show that one-level integration provides significant
improvements for all trackers: DCF’s average rank improves from 18th to 17th, ECOgpu from 16th to 10th, ECOhc from 15th to 5th and CSR-DCF from 19th to 14th. CSR-DCF with two-level integration achieved
the top rank by a clear margin on PTB. Our framework is particularly powerful in occlusion scenarios where it provides 13.5% average improvement and 26% for the best tracker (CSR-DCF).
