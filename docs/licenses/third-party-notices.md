## 算法移植的许可证与版权声明

以下声明适用于对应算法及衍生实现；各自的许可证和 Rust 文件中的署名继续保留。
其中 OKVIS2-X 的声明覆盖保留的 IMU 传播衍生实现；历史来源署名亦保留用于追溯。

### OKVIS2-X

```text
OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
Depth or LiDAR, and GNSS
Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
Copyright (c) 2020, Smart Robotics Lab / Imperial College London
Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
and ETH Zurich

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Autonomous Systems Lab / ETH Zurich, Smart Robotics Lab /
  Imperial College London, Mobile Robotics Lab / Technical University of Munich
  and ETH Zurich, nor the names of its contributors may be used to endorse or
  promote products derived from this software without specific prior written
  permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Authors and contributors:
- Simon Boche
- Jaehyung Jung
- Sebastián Barbas Laina
- Leonard Freissmuth
- Micha Bosshart
- Sotiris Papatheodorou
- Simon Schaefer
- Stefan Leutenegger
- Andreas Forster
- Paul Furgale
- Pascal Gohl
- Simon Lynen
```

### BRISK / AGAST

```text
Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 Copyright (C) 2020  Smart Robotics Lab, Imperial College London
 Stefan Leutenegger

 Copyright (C) 2024  Smart Robotics Lab, Technical University of Munich
 Stefan Leutenegger

 All rights reserved.

 This is the Author's implementation of BRISK: Binary Robust Invariant
 Scalable Keypoints [1]. Various (partly unpublished) extensions are provided,
 some of which are described in [2].

 [1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK: Binary
     Robust Invariant Scalable Keypoints. In Proceedings of the IEEE
     International Conference on Computer Vision (ICCV), 2011.

 [2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for
     Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

 This file is part of BRISK.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### DBoW2 / DUtils

```text
DBoW2: bag-of-words library for C++ with generic descriptors

Copyright (c) 2015 Dorian Galvez-Lopez. http://doriangalvez.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The original author of the work must be notified of any
   redistribution of source code or in binary form.
4. Neither the name of copyright holders nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

If you use it in an academic work, please cite:

  @ARTICLE{GalvezTRO12,
    author={G\'alvez-L\'opez, Dorian and Tard\'os, J. D.},
    journal={IEEE Transactions on Robotics},
    title={Bags of Binary Words for Fast Place Recognition in Image Sequences},
    year={2012},
    month={October},
    volume={28},
    number={5},
    pages={1188--1197},
    doi={10.1109/TRO.2012.2197158},
    ISSN={1552-3098}
  }
```

### OpenGV

```text
/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/
```

### SRL projection

```text
OKVIS - Open Keyframe-based Visual-Inertial SLAM
 Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 Copyright (c) 2020, Smart Robotics Lab / Imperial College London

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
    its contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
```

### Supereight2

```text
Copyright (c) <year> <owner>. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### tinycolormap / Turbo

```text
MIT License

 Copyright (c) 2018-2020 Yuki Koyama

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 -------------------------------------------------------------------------------

 The lookup table for Turbo is derived by Shot511 in his PR,
 https://github.com/yuki-koyama/tinycolormap/pull/27 , from
 https://gist.github.com/mikhailov-work/6a308c20e494d9e0ccc29036b28faa7a , which
 is released by Anton Mikhailov, copyrighted by Google LLC, and licensed under
 the Apache 2.0 license. To the best of our knowledge, the Apache 2.0 license is
 compatible with the MIT license, and thus we release the merged entire code
 under the MIT license. The license notice for Anton's code is posted here:

 Copyright 2019 Google LLC.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
     http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
```

### Ceres 算法衍生实现

Rust 图求解器中的 Ceres 算法衍生实现保留以下上游声明；不依赖 Ceres 库。

```text
Ceres Solver - A fast non-linear least squares minimizer
Copyright 2023 Google Inc. All rights reserved.
http://ceres-solver.org/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Google Inc. nor the names of its contributors may be
  used to endorse or promote products derived from this software without
  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------------------------------------------------------

                              Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION

   1. Definitions.

      "License" shall mean the terms and conditions for use, reproduction,
      and distribution as defined by Sections 1 through 9 of this document.

      "Licensor" shall mean the copyright owner or entity authorized by
      the copyright owner that is granting the License.

      "Legal Entity" shall mean the union of the acting entity and all
      other entities that control, are controlled by, or are under common
      control with that entity. For the purposes of this definition,
      "control" means (i) the power, direct or indirect, to cause the
      direction or management of such entity, whether by contract or
      otherwise, or (ii) ownership of fifty percent (50%) or more of the
      outstanding shares, or (iii) beneficial ownership of such entity.

      "You" (or "Your") shall mean an individual or Legal Entity
      exercising permissions granted by this License.

      "Source" form shall mean the preferred form for making modifications,
      including but not limited to software source code, documentation
      source, and configuration files.

      "Object" form shall mean any form resulting from mechanical
      transformation or translation of a Source form, including but
      not limited to compiled object code, generated documentation,
      and conversions to other media types.

      "Work" shall mean the work of authorship, whether in Source or
      Object form, made available under the License, as indicated by a
      copyright notice that is included in or attached to the work
      (an example is provided in the Appendix below).

      "Derivative Works" shall mean any work, whether in Source or Object
      form, that is based on (or derived from) the Work and for which the
      editorial revisions, annotations, elaborations, or other modifications
      represent, as a whole, an original work of authorship. For the purposes
      of this License, Derivative Works shall not include works that remain
      separable from, or merely link (or bind by name) to the interfaces of,
      the Work and Derivative Works thereof.

      "Contribution" shall mean any work of authorship, including
      the original version of the Work and any modifications or additions
      to that Work or Derivative Works thereof, that is intentionally
      submitted to Licensor for inclusion in the Work by the copyright owner
      or by an individual or Legal Entity authorized to submit on behalf of
      the copyright owner. For the purposes of this definition, "submitted"
      means any form of electronic, verbal, or written communication sent
      to the Licensor or its representatives, including but not limited to
      communication on electronic mailing lists, source code control systems,
      and issue tracking systems that are managed by, or on behalf of, the
      Licensor for the purpose of discussing and improving the Work, but
      excluding communication that is conspicuously marked or otherwise
      designated in writing by the copyright owner as "Not a Contribution."

      "Contributor" shall mean Licensor and any individual or Legal Entity
      on behalf of whom a Contribution has been received by Licensor and
      subsequently incorporated within the Work.

   2. Grant of Copyright License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      copyright license to reproduce, prepare Derivative Works of,
      publicly display, publicly perform, sublicense, and distribute the
      Work and such Derivative Works in Source or Object form.

   3. Grant of Patent License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      (except as stated in this section) patent license to make, have made,
      use, offer to sell, sell, import, and otherwise transfer the Work,
      where such license applies only to those patent claims licensable
      by such Contributor that are necessarily infringed by their
      Contribution(s) alone or by combination of their Contribution(s)
      with the Work to which such Contribution(s) was submitted. If You
      institute patent litigation against any entity (including a
      cross-claim or counterclaim in a lawsuit) alleging that the Work
      or a Contribution incorporated within the Work constitutes direct
      or contributory patent infringement, then any patent licenses
      granted to You under this License for that Work shall terminate
      as of the date such litigation is filed.

   4. Redistribution. You may reproduce and distribute copies of the
      Work or Derivative Works thereof in any medium, with or without
      modifications, and in Source or Object form, provided that You
      meet the following conditions:

      (a) You must give any other recipients of the Work or
          Derivative Works a copy of this License; and

      (b) You must cause any modified files to carry prominent notices
          stating that You changed the files; and

      (c) You must retain, in the Source form of any Derivative Works
          that You distribute, all copyright, patent, trademark, and
          attribution notices from the Source form of the Work,
          excluding those notices that do not pertain to any part of
          the Derivative Works; and

      (d) If the Work includes a "NOTICE" text file as part of its
          distribution, then any Derivative Works that You distribute must
          include a readable copy of the attribution notices contained
          within such NOTICE file, excluding those notices that do not
          pertain to any part of the Derivative Works, in at least one
          of the following places: within a NOTICE text file distributed
          as part of the Derivative Works; within the Source form or
          documentation, if provided along with the Derivative Works; or,
          within a display generated by the Derivative Works, if and
          wherever such third-party notices normally appear. The contents
          of the NOTICE file are for informational purposes only and
          do not modify the License. You may add Your own attribution
          notices within Derivative Works that You distribute, alongside
          or as an addendum to the NOTICE text from the Work, provided
          that such additional attribution notices cannot be construed
          as modifying the License.

      You may add Your own copyright statement to Your modifications and
      may provide additional or different license terms and conditions
      for use, reproduction, or distribution of Your modifications, or
      for any such Derivative Works as a whole, provided Your use,
      reproduction, and distribution of the Work otherwise complies with
      the conditions stated in this License.

   5. Submission of Contributions. Unless You explicitly state otherwise,
      any Contribution intentionally submitted for inclusion in the Work
      by You to the Licensor shall be under the terms and conditions of
      this License, without any additional terms or conditions.
      Notwithstanding the above, nothing herein shall supersede or modify
      the terms of any separate license agreement you may have executed
      with Licensor regarding such Contributions.

   6. Trademarks. This License does not grant permission to use the trade
      names, trademarks, service marks, or product names of the Licensor,
      except as required for reasonable and customary use in describing the
      origin of the Work and reproducing the content of the NOTICE file.

   7. Disclaimer of Warranty. Unless required by applicable law or
      agreed to in writing, Licensor provides the Work (and each
      Contributor provides its Contributions) on an "AS IS" BASIS,
      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
      implied, including, without limitation, any warranties or conditions
      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
      PARTICULAR PURPOSE. You are solely responsible for determining the
      appropriateness of using or redistributing the Work and assume any
      risks associated with Your exercise of permissions under this License.

   8. Limitation of Liability. In no event and under no legal theory,
      whether in tort (including negligence), contract, or otherwise,
      unless required by applicable law (such as deliberate and grossly
      negligent acts) or agreed to in writing, shall any Contributor be
      liable to You for damages, including any direct, indirect, special,
      incidental, or consequential damages of any character arising as a
      result of this License or out of the use or inability to use the
      Work (including but not limited to damages for loss of goodwill,
      work stoppage, computer failure or malfunction, or any and all
      other commercial damages or losses), even if such Contributor
      has been advised of the possibility of such damages.

   9. Accepting Warranty or Additional Liability. While redistributing
      the Work or Derivative Works thereof, You may choose to offer,
      and charge a fee for, acceptance of support, warranty, indemnity,
      or other liability obligations and/or rights consistent with this
      License. However, in accepting such obligations, You may act only
      on Your own behalf and on Your sole responsibility, not on behalf
      of any other Contributor, and only if You agree to indemnify,
      defend, and hold each Contributor harmless for any liability
      incurred by, or claims asserted against, such Contributor by reason
      of your accepting any such warranty or additional liability.

   END OF TERMS AND CONDITIONS

   APPENDIX: How to apply the Apache License to your work.

      To apply the Apache License to your work, attach the following
      boilerplate notice, with the fields enclosed by brackets "[]"
      replaced with your own identifying information. (Don't include
      the brackets!)  The text should be enclosed in the appropriate
      comment syntax for the file format. We also recommend that a
      file or class name and description of purpose be included on the
      same "printed page" as the copyright notice for easier
      identification within third-party archives.

   Copyright [yyyy] [name of copyright owner]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

-------------------------------------------------------------------------------------------------------------------------------

Some of the code in the examples directory derives from libmv, which is
distributed under the MIT license as described below


Copyright (c) 2007-2011 libmv authors.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
```

### Eigen / MINPACK LM

`firefly-opengv/src/optimization_functor/minpack_lm.rs` 采用 MPL-2.0。

```text
Copyright (C) 2009 Thomas Capricelli <orzel@freehackers.org>
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
```

### 上游文件中的版权署名

```text
" Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.";
Ceres Solver, Copyright 2023 Google Inc. All rights reserved.
Copyright (C) 2009 Thomas Capricelli <orzel@freehackers.org>
Copyright (C) 2010  Elmar Mair
Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
Copyright (C) 2020  Smart Robotics Lab, Imperial College London
Copyright (C) 2024  Smart Robotics Lab, Technical University of Munich
Copyright (c) 2011-2013, Paul Furgale and others.
Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
Copyright (c) 2018-2020 Yuki Koyama
Copyright (c) 2020, Smart Robotics Lab / Imperial College London
Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
Copyright (c) 2026, Mobile Robotics Lab / Technical University of Munich
Copyright 2019 Google LLC.
DBoW2 — Copyright (c) 2011–2013 Dorian Galvez-Lopez.
FindAnything / OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.
License: Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.
OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
Original Copyright (c) 2008, Willow Garage, Inc.
Original Copyright (c) 2010, Willow Garage, Inc.
SPDX-FileCopyrightText: 2009-2011 NVIDIA Corporation
SPDX-FileCopyrightText: 2011-2013 Gerhard Reitmayr, TU Graz
SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
SPDX-FileCopyrightText: 2018-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2019 Emanuele Vespa
SPDX-FileCopyrightText: 2019-2021 Nils Funk
SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2019-2023 Nils Funk
SPDX-FileCopyrightText: 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2019-2023 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2019-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2019-2025 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2020 Marija Popovic
SPDX-FileCopyrightText: 2020-2021 Nils Funk
SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2020-2022 Nils Funk
SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2020-2023 Nils Funk
SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2020-2023 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2020-2024 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2021 Nils Funk
SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2021-2022 Nils Funk
SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2021-2023 Nils Funk
SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2021-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2021-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2021-2025 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2022 Nils Funk
SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2022-2024 Simon Boche
SPDX-FileCopyrightText: 2022-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2022-2024 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2023-2024 Simon Boche
SPDX-FileCopyrightText: 2023-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2023-2024 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2024-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2024-2025 Sotiris Papatheodorou
SPDX-FileCopyrightText: 2025 Mobile Robotics Lab, ETH Zurich, Technical University of Munich
SPDX-FileCopyrightText: 2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
SPDX-FileCopyrightText: 2025 Sotiris Papatheodorou
```
