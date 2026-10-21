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

use super::{
    reductors::{
        groebner_row30_000000000_f, groebner_row30_010000000_f, groebner_row30_100000000_f,
        groebner_row31_000000000_f, groebner_row31_010000000_f, groebner_row31_100000000_f,
        groebner_row32_000000000_f, groebner_row32_000100000_f, groebner_row32_010000000_f,
        groebner_row32_100000000_f, groebner_row33_000000000_f, groebner_row33_000100000_f,
        groebner_row33_010000000_f, groebner_row33_100000000_f, groebner_row34_000000000_f,
        groebner_row34_000100000_f, groebner_row34_100000000_f, groebner_row35_000000000_f,
        groebner_row35_000100000_f, groebner_row35_100000000_f, groebner_row36_000000000_f,
        groebner_row36_000100000_f, groebner_row36_100000000_f, groebner_row37_000000000_f,
        groebner_row37_000100000_f, groebner_row37_100000000_f, groebner_row38_000000000_f,
        groebner_row38_000100000_f, groebner_row38_100000000_f, groebner_row39_000000000_f,
        groebner_row39_000100000_f, groebner_row39_100000000_f, groebner_row40_000000000_f,
        groebner_row41_000000000_f, groebner_row42_000000000_f, groebner_row43_000000000_f,
        groebner_row44_000000000_f, groebner_row45_000000000_f, groebner_row46_000000000_f,
        groebner_row47_000000000_f, groebner_row48_000000000_f, groebner_row49_000000000_f,
        groebner_row50_000000000_f, groebner_row51_000000000_f, groebner_row52_000000000_f,
        groebner_row53_000000000_f, groebner_row54_000000000_f, groebner_row55_000000000_f,
        groebner_row56_000000000_f, groebner_row57_000000000_f, groebner_row58_000000000_f,
        groebner_row59_000000000_f, groebner_row60_000000000_f, groebner_row61_000000000_f,
        groebner_row61_010000000_f, groebner_row61_100000000_f, groebner_row62_000000000_f,
        groebner_row62_010000000_f, groebner_row62_100000000_f, groebner_row63_000000000_f,
        groebner_row63_010000000_f, groebner_row63_100000000_f, groebner_row64_000000000_f,
        groebner_row64_010000000_f, groebner_row64_100000000_f,
    },
    spolynomials::{
        s_polynomial30, s_polynomial31, s_polynomial32, s_polynomial33, s_polynomial34,
        s_polynomial35, s_polynomial36, s_polynomial37, s_polynomial38, s_polynomial39,
        s_polynomial40, s_polynomial41, s_polynomial42, s_polynomial43, s_polynomial44,
        s_polynomial45, s_polynomial46, s_polynomial47, s_polynomial48, s_polynomial49,
        s_polynomial50, s_polynomial51, s_polynomial52, s_polynomial53, s_polynomial54,
        s_polynomial55, s_polynomial56, s_polynomial57, s_polynomial58, s_polynomial59,
        s_polynomial60, s_polynomial61, s_polynomial62, s_polynomial63, s_polynomial64,
        s_polynomial65,
    },
};
use nalgebra::SMatrix;
pub fn compute_basis(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    compute_basis_section_0(groebner_matrix);
    compute_basis_section_1(groebner_matrix);
    compute_basis_section_2(groebner_matrix);
    compute_basis_section_3(groebner_matrix);
}
#[inline]
fn compute_basis_section_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    compute_basis_stage_0(groebner_matrix);
    compute_basis_stage_1(groebner_matrix);
    compute_basis_stage_2(groebner_matrix);
    compute_basis_stage_3(groebner_matrix);
    compute_basis_stage_4(groebner_matrix);
    compute_basis_stage_5(groebner_matrix);
    compute_basis_stage_6(groebner_matrix);
    compute_basis_stage_7(groebner_matrix);
    compute_basis_stage_8(groebner_matrix);
    compute_basis_stage_9(groebner_matrix);
    compute_basis_stage_10(groebner_matrix);
    compute_basis_stage_11(groebner_matrix);
    compute_basis_stage_12(groebner_matrix);
    compute_basis_stage_13(groebner_matrix);
    compute_basis_stage_14(groebner_matrix);
    compute_basis_stage_15(groebner_matrix);
    compute_basis_stage_16(groebner_matrix);
    compute_basis_stage_17(groebner_matrix);
    compute_basis_stage_18(groebner_matrix);
    compute_basis_stage_19(groebner_matrix);
    compute_basis_stage_20(groebner_matrix);
    compute_basis_stage_21(groebner_matrix);
    compute_basis_stage_22(groebner_matrix);
    compute_basis_stage_23(groebner_matrix);
    compute_basis_stage_24(groebner_matrix);
    compute_basis_stage_25(groebner_matrix);
    compute_basis_stage_26(groebner_matrix);
    compute_basis_stage_27(groebner_matrix);
    compute_basis_stage_28(groebner_matrix);
    compute_basis_stage_29(groebner_matrix);
    compute_basis_stage_30(groebner_matrix);
    compute_basis_stage_31(groebner_matrix);
    compute_basis_stage_32(groebner_matrix);
    compute_basis_stage_33(groebner_matrix);
    compute_basis_stage_34(groebner_matrix);
    compute_basis_stage_35(groebner_matrix);
    compute_basis_stage_36(groebner_matrix);
    compute_basis_stage_37(groebner_matrix);
    compute_basis_stage_38(groebner_matrix);
    compute_basis_stage_39(groebner_matrix);
    compute_basis_stage_40(groebner_matrix);
    compute_basis_stage_41(groebner_matrix);
    compute_basis_stage_42(groebner_matrix);
    compute_basis_stage_43(groebner_matrix);
    compute_basis_stage_44(groebner_matrix);
    compute_basis_stage_45(groebner_matrix);
    compute_basis_stage_46(groebner_matrix);
    compute_basis_stage_47(groebner_matrix);
    compute_basis_stage_48(groebner_matrix);
    compute_basis_stage_49(groebner_matrix);
    compute_basis_stage_50(groebner_matrix);
    compute_basis_stage_51(groebner_matrix);
    compute_basis_stage_52(groebner_matrix);
    compute_basis_stage_53(groebner_matrix);
    compute_basis_stage_54(groebner_matrix);
    compute_basis_stage_55(groebner_matrix);
    compute_basis_stage_56(groebner_matrix);
    compute_basis_stage_57(groebner_matrix);
    compute_basis_stage_58(groebner_matrix);
    compute_basis_stage_59(groebner_matrix);
    compute_basis_stage_60(groebner_matrix);
    compute_basis_stage_61(groebner_matrix);
}

#[inline]
fn compute_basis_section_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    compute_basis_stage_62(groebner_matrix);
    compute_basis_stage_63(groebner_matrix);
    compute_basis_stage_64(groebner_matrix);
    compute_basis_stage_65(groebner_matrix);
    compute_basis_stage_66(groebner_matrix);
    compute_basis_stage_67(groebner_matrix);
    compute_basis_stage_68(groebner_matrix);
    compute_basis_stage_69(groebner_matrix);
    compute_basis_stage_70(groebner_matrix);
    compute_basis_stage_71(groebner_matrix);
    compute_basis_stage_72(groebner_matrix);
    compute_basis_stage_73(groebner_matrix);
    compute_basis_stage_74(groebner_matrix);
    compute_basis_stage_75(groebner_matrix);
    compute_basis_stage_76(groebner_matrix);
    compute_basis_stage_77(groebner_matrix);
    compute_basis_stage_78(groebner_matrix);
    compute_basis_stage_79(groebner_matrix);
    compute_basis_stage_80(groebner_matrix);
    compute_basis_stage_81(groebner_matrix);
    compute_basis_stage_82(groebner_matrix);
    compute_basis_stage_83(groebner_matrix);
    compute_basis_stage_84(groebner_matrix);
    compute_basis_stage_85(groebner_matrix);
    compute_basis_stage_86(groebner_matrix);
    compute_basis_stage_87(groebner_matrix);
    compute_basis_stage_88(groebner_matrix);
    compute_basis_stage_89(groebner_matrix);
    compute_basis_stage_90(groebner_matrix);
    compute_basis_stage_91(groebner_matrix);
    compute_basis_stage_92(groebner_matrix);
    compute_basis_stage_93(groebner_matrix);
    compute_basis_stage_94(groebner_matrix);
    compute_basis_stage_95(groebner_matrix);
    compute_basis_stage_96(groebner_matrix);
    compute_basis_stage_97(groebner_matrix);
    compute_basis_stage_98(groebner_matrix);
    compute_basis_stage_99(groebner_matrix);
    compute_basis_stage_100(groebner_matrix);
    compute_basis_stage_101(groebner_matrix);
    compute_basis_stage_102(groebner_matrix);
    compute_basis_stage_103(groebner_matrix);
    compute_basis_stage_104(groebner_matrix);
    compute_basis_stage_105(groebner_matrix);
    compute_basis_stage_106(groebner_matrix);
    compute_basis_stage_107(groebner_matrix);
    compute_basis_stage_108(groebner_matrix);
    compute_basis_stage_109(groebner_matrix);
    compute_basis_stage_110(groebner_matrix);
    compute_basis_stage_111(groebner_matrix);
    compute_basis_stage_112(groebner_matrix);
    compute_basis_stage_113(groebner_matrix);
    compute_basis_stage_114(groebner_matrix);
    compute_basis_stage_115(groebner_matrix);
    compute_basis_stage_116(groebner_matrix);
    compute_basis_stage_117(groebner_matrix);
    compute_basis_stage_118(groebner_matrix);
    compute_basis_stage_119(groebner_matrix);
    compute_basis_stage_120(groebner_matrix);
    compute_basis_stage_121(groebner_matrix);
    compute_basis_stage_122(groebner_matrix);
    compute_basis_stage_123(groebner_matrix);
}

#[inline]
fn compute_basis_section_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    compute_basis_stage_124(groebner_matrix);
    compute_basis_stage_125(groebner_matrix);
    compute_basis_stage_126(groebner_matrix);
    compute_basis_stage_127(groebner_matrix);
    compute_basis_stage_128(groebner_matrix);
    compute_basis_stage_129(groebner_matrix);
    compute_basis_stage_130(groebner_matrix);
    compute_basis_stage_131(groebner_matrix);
    compute_basis_stage_132(groebner_matrix);
    compute_basis_stage_133(groebner_matrix);
    compute_basis_stage_134(groebner_matrix);
    compute_basis_stage_135(groebner_matrix);
    compute_basis_stage_136(groebner_matrix);
    compute_basis_stage_137(groebner_matrix);
    compute_basis_stage_138(groebner_matrix);
    compute_basis_stage_139(groebner_matrix);
    compute_basis_stage_140(groebner_matrix);
    compute_basis_stage_141(groebner_matrix);
    compute_basis_stage_142(groebner_matrix);
    compute_basis_stage_143(groebner_matrix);
    compute_basis_stage_144(groebner_matrix);
    compute_basis_stage_145(groebner_matrix);
    compute_basis_stage_146(groebner_matrix);
    compute_basis_stage_147(groebner_matrix);
    compute_basis_stage_148(groebner_matrix);
    compute_basis_stage_149(groebner_matrix);
    compute_basis_stage_150(groebner_matrix);
    compute_basis_stage_151(groebner_matrix);
    compute_basis_stage_152(groebner_matrix);
    compute_basis_stage_153(groebner_matrix);
    compute_basis_stage_154(groebner_matrix);
    compute_basis_stage_155(groebner_matrix);
    compute_basis_stage_156(groebner_matrix);
    compute_basis_stage_157(groebner_matrix);
    compute_basis_stage_158(groebner_matrix);
    compute_basis_stage_159(groebner_matrix);
    compute_basis_stage_160(groebner_matrix);
    compute_basis_stage_161(groebner_matrix);
    compute_basis_stage_162(groebner_matrix);
    compute_basis_stage_163(groebner_matrix);
    compute_basis_stage_164(groebner_matrix);
    compute_basis_stage_165(groebner_matrix);
    compute_basis_stage_166(groebner_matrix);
    compute_basis_stage_167(groebner_matrix);
    compute_basis_stage_168(groebner_matrix);
    compute_basis_stage_169(groebner_matrix);
    compute_basis_stage_170(groebner_matrix);
    compute_basis_stage_171(groebner_matrix);
    compute_basis_stage_172(groebner_matrix);
    compute_basis_stage_173(groebner_matrix);
    compute_basis_stage_174(groebner_matrix);
    compute_basis_stage_175(groebner_matrix);
    compute_basis_stage_176(groebner_matrix);
    compute_basis_stage_177(groebner_matrix);
    compute_basis_stage_178(groebner_matrix);
    compute_basis_stage_179(groebner_matrix);
    compute_basis_stage_180(groebner_matrix);
    compute_basis_stage_181(groebner_matrix);
    compute_basis_stage_182(groebner_matrix);
    compute_basis_stage_183(groebner_matrix);
    compute_basis_stage_184(groebner_matrix);
    compute_basis_stage_185(groebner_matrix);
}

#[inline]
fn compute_basis_section_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    compute_basis_stage_186(groebner_matrix);
    compute_basis_stage_187(groebner_matrix);
    compute_basis_stage_188(groebner_matrix);
    compute_basis_stage_189(groebner_matrix);
    compute_basis_stage_190(groebner_matrix);
    compute_basis_stage_191(groebner_matrix);
    compute_basis_stage_192(groebner_matrix);
    compute_basis_stage_193(groebner_matrix);
    compute_basis_stage_194(groebner_matrix);
    compute_basis_stage_195(groebner_matrix);
    compute_basis_stage_196(groebner_matrix);
    compute_basis_stage_197(groebner_matrix);
    compute_basis_stage_198(groebner_matrix);
    compute_basis_stage_199(groebner_matrix);
    compute_basis_stage_200(groebner_matrix);
    compute_basis_stage_201(groebner_matrix);
    compute_basis_stage_202(groebner_matrix);
    compute_basis_stage_203(groebner_matrix);
    compute_basis_stage_204(groebner_matrix);
    compute_basis_stage_205(groebner_matrix);
    compute_basis_stage_206(groebner_matrix);
    compute_basis_stage_207(groebner_matrix);
    compute_basis_stage_208(groebner_matrix);
    compute_basis_stage_209(groebner_matrix);
    compute_basis_stage_210(groebner_matrix);
    compute_basis_stage_211(groebner_matrix);
    compute_basis_stage_212(groebner_matrix);
    compute_basis_stage_213(groebner_matrix);
    compute_basis_stage_214(groebner_matrix);
    compute_basis_stage_215(groebner_matrix);
    compute_basis_stage_216(groebner_matrix);
    compute_basis_stage_217(groebner_matrix);
    compute_basis_stage_218(groebner_matrix);
    compute_basis_stage_219(groebner_matrix);
    compute_basis_stage_220(groebner_matrix);
    compute_basis_stage_221(groebner_matrix);
    compute_basis_stage_222(groebner_matrix);
    compute_basis_stage_223(groebner_matrix);
    compute_basis_stage_224(groebner_matrix);
    compute_basis_stage_225(groebner_matrix);
    compute_basis_stage_226(groebner_matrix);
}

#[inline]
fn compute_basis_stage_0(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial30(groebner_matrix);
    let factor = -groebner_matrix[(30, 1)];
    groebner_matrix[(30, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(30, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(30, 179)] -= reduction_term;
    let factor = groebner_matrix[(30, 2)];
    groebner_matrix[(30, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(30, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(30, 24)] -= reduction_term;
    let factor = -groebner_matrix[(30, 3)];
    groebner_matrix[(30, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(30, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(30, 180)] -= reduction_term;
    let factor = groebner_matrix[(30, 4)];
    groebner_matrix[(30, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(30, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(30, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(30, 190)] -= reduction_term;
    let factor = groebner_matrix[(30, 5)];
    groebner_matrix[(30, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(30, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(30, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(30, 188)] -= reduction_term;
    let factor = groebner_matrix[(30, 6)];
    groebner_matrix[(30, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(30, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(30, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(30, 189)] -= reduction_term;
    let factor = -groebner_matrix[(30, 10)];
    groebner_matrix[(30, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 163)] -= reduction_term;
    let factor = -groebner_matrix[(30, 11)];
    groebner_matrix[(30, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 164)] -= reduction_term;
    let factor = -groebner_matrix[(30, 12)];
    groebner_matrix[(30, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 165)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_1(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 13)];

    groebner_matrix[(30, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(30, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(30, 30)] -= reduction_term;
    let factor = -groebner_matrix[(30, 14)];
    groebner_matrix[(30, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(30, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(30, 182)] -= reduction_term;
    let factor = -groebner_matrix[(30, 15)];
    groebner_matrix[(30, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 166)] -= reduction_term;
    let factor = -groebner_matrix[(30, 18)];
    groebner_matrix[(30, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 167)] -= reduction_term;
    let factor = groebner_matrix[(30, 19)];
    groebner_matrix[(30, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(30, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(30, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(30, 192)] -= reduction_term;
    let factor = groebner_matrix[(30, 22)];
    groebner_matrix[(30, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(30, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(30, 123)] -= reduction_term;
    let factor = groebner_matrix[(30, 25)];
    groebner_matrix[(30, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(30, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(30, 35)] -= reduction_term;
    let factor = -groebner_matrix[(30, 26)];
    groebner_matrix[(30, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(30, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(30, 183)] -= reduction_term;
    let factor = groebner_matrix[(30, 27)];
    groebner_matrix[(30, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(30, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(30, 175)] -= reduction_term;
    let factor = -groebner_matrix[(30, 30)];
    groebner_matrix[(30, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 168)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_2(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 39)];

    groebner_matrix[(30, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(30, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(30, 101)] -= reduction_term;
    let factor = groebner_matrix[(30, 40)];
    groebner_matrix[(30, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(30, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(30, 158)] -= reduction_term;
    let factor = groebner_matrix[(30, 41)];
    groebner_matrix[(30, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(30, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(30, 102)] -= reduction_term;
    let factor = groebner_matrix[(30, 42)];
    groebner_matrix[(30, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(30, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(30, 159)] -= reduction_term;
    let factor = groebner_matrix[(30, 43)];
    groebner_matrix[(30, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(30, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(30, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(30, 193)] -= reduction_term;
    let factor = groebner_matrix[(30, 44)];
    groebner_matrix[(30, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(30, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(30, 113)] -= reduction_term;
    let factor = groebner_matrix[(30, 45)];
    groebner_matrix[(30, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(30, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(30, 114)] -= reduction_term;
    let factor = groebner_matrix[(30, 46)];
    groebner_matrix[(30, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(30, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(30, 109)] -= reduction_term;
    let factor = groebner_matrix[(30, 47)];
    groebner_matrix[(30, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(30, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(30, 161)] -= reduction_term;
    let factor = -groebner_matrix[(30, 48)];
    groebner_matrix[(30, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 169)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_3(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 49)];

    groebner_matrix[(30, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(30, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(30, 116)] -= reduction_term;
    let factor = -groebner_matrix[(30, 50)];
    groebner_matrix[(30, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(30, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(30, 154)] -= reduction_term;
    let factor = groebner_matrix[(30, 51)];
    groebner_matrix[(30, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(30, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(30, 114)] -= reduction_term;
    let factor = groebner_matrix[(30, 52)];
    groebner_matrix[(30, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(30, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(30, 162)] -= reduction_term;
    let factor = groebner_matrix[(30, 53)];
    groebner_matrix[(30, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(30, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(30, 146)] -= reduction_term;
    let factor = groebner_matrix[(30, 54)];
    groebner_matrix[(30, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(30, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(30, 117)] -= reduction_term;
    let factor = -groebner_matrix[(30, 55)];
    groebner_matrix[(30, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(30, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(30, 159)] -= reduction_term;
    let factor = groebner_matrix[(30, 56)];
    groebner_matrix[(30, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(30, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(30, 158)] -= reduction_term;
    let factor = groebner_matrix[(30, 57)];
    groebner_matrix[(30, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(30, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(30, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(30, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_4(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 58)];

    groebner_matrix[(30, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(30, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(30, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(30, 189)] -= reduction_term;
    let factor = groebner_matrix[(30, 59)];
    groebner_matrix[(30, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(30, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(30, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(30, 191)] -= reduction_term;
    let factor = groebner_matrix[(30, 60)];
    groebner_matrix[(30, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(30, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(30, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(30, 192)] -= reduction_term;
    let factor = groebner_matrix[(30, 61)];
    groebner_matrix[(30, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(30, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(30, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(30, 194)] -= reduction_term;
    let factor = groebner_matrix[(30, 64)];
    groebner_matrix[(30, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 148)] -= reduction_term;
    let factor = groebner_matrix[(30, 65)];
    groebner_matrix[(30, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 149)] -= reduction_term;
    let factor = groebner_matrix[(30, 66)];
    groebner_matrix[(30, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 150)] -= reduction_term;
    let factor = groebner_matrix[(30, 67)];
    groebner_matrix[(30, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(30, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(30, 78)] -= reduction_term;
    let factor = -groebner_matrix[(30, 68)];
    groebner_matrix[(30, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(30, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(30, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_5(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 69)];

    groebner_matrix[(30, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 151)] -= reduction_term;
    let factor = groebner_matrix[(30, 70)];
    groebner_matrix[(30, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(30, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(30, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(30, 194)] -= reduction_term;
    let factor = -groebner_matrix[(30, 73)];
    groebner_matrix[(30, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 176)] -= reduction_term;
    let factor = -groebner_matrix[(30, 76)];
    groebner_matrix[(30, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 142)] -= reduction_term;
    let factor = -groebner_matrix[(30, 77)];
    groebner_matrix[(30, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 143)] -= reduction_term;
    let factor = groebner_matrix[(30, 78)];
    groebner_matrix[(30, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 160)] -= reduction_term;
    let factor = -groebner_matrix[(30, 79)];
    groebner_matrix[(30, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 148)] -= reduction_term;
    let factor = -groebner_matrix[(30, 80)];
    groebner_matrix[(30, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 152)] -= reduction_term;
    let factor = -groebner_matrix[(30, 81)];
    groebner_matrix[(30, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 157)] -= reduction_term;
    let factor = groebner_matrix[(30, 82)];
    groebner_matrix[(30, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(30, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(30, 127)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_6(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 83)];

    groebner_matrix[(30, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(30, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(30, 175)] -= reduction_term;
    let factor = groebner_matrix[(30, 84)];
    groebner_matrix[(30, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 166)] -= reduction_term;
    let factor = groebner_matrix[(30, 85)];
    groebner_matrix[(30, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(30, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(30, 130)] -= reduction_term;
    let factor = -groebner_matrix[(30, 86)];
    groebner_matrix[(30, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(30, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(30, 172)] -= reduction_term;
    let factor = -groebner_matrix[(30, 87)];
    groebner_matrix[(30, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 163)] -= reduction_term;
    let factor = groebner_matrix[(30, 91)];
    groebner_matrix[(30, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 173)] -= reduction_term;
    let factor = -groebner_matrix[(30, 94)];
    groebner_matrix[(30, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 170)] -= reduction_term;
    let factor = groebner_matrix[(30, 97)];
    groebner_matrix[(30, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(30, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(30, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(30, 195)] -= reduction_term;
    let factor = groebner_matrix[(30, 100)];
    groebner_matrix[(30, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(30, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(30, 139)] -= reduction_term;
    let factor = groebner_matrix[(30, 103)];
    groebner_matrix[(30, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(30, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(30, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_7(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(30, 104)];

    groebner_matrix[(30, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(30, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(30, 186)] -= reduction_term;
    let factor = groebner_matrix[(30, 105)];
    groebner_matrix[(30, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(30, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(30, 185)] -= reduction_term;
    let factor = groebner_matrix[(30, 106)];
    groebner_matrix[(30, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(30, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(30, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(30, 195)] -= reduction_term;
    let factor = -groebner_matrix[(30, 109)];
    groebner_matrix[(30, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 184)] -= reduction_term;
    let factor = groebner_matrix[(30, 118)];
    groebner_matrix[(30, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(30, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(30, 135)] -= reduction_term;
    let factor = groebner_matrix[(30, 119)];
    groebner_matrix[(30, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(30, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(30, 183)] -= reduction_term;
    let factor = -groebner_matrix[(30, 120)];
    groebner_matrix[(30, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(30, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(30, 182)] -= reduction_term;
    let factor = groebner_matrix[(30, 121)];
    groebner_matrix[(30, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(30, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(30, 138)] -= reduction_term;
    let factor = -groebner_matrix[(30, 122)];
    groebner_matrix[(30, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(30, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(30, 180)] -= reduction_term;
    let factor = groebner_matrix[(30, 123)];
    groebner_matrix[(30, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(30, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(30, 179)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_8(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 127)];

    groebner_matrix[(30, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 181)] -= reduction_term;
    let factor = -groebner_matrix[(30, 130)];
    groebner_matrix[(30, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 178)] -= reduction_term;
    let factor = groebner_matrix[(30, 142)];
    groebner_matrix[(30, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(30, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(30, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(30, 196)] -= reduction_term;
    let factor = groebner_matrix[(30, 143)];
    groebner_matrix[(30, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(30, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(30, 176)] -= reduction_term;
    let factor = groebner_matrix[(30, 144)];
    groebner_matrix[(30, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(30, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(30, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(30, 196)] -= reduction_term;
    let factor = groebner_matrix[(30, 146)];
    groebner_matrix[(30, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(30, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(30, 185)] -= reduction_term;
    let factor = groebner_matrix[(30, 147)];
    groebner_matrix[(30, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(30, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(30, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(30, 196)] -= reduction_term;
    let factor = groebner_matrix[(30, 148)];
    groebner_matrix[(30, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(30, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(30, 159)] -= reduction_term;
    let factor = -groebner_matrix[(30, 149)];
    groebner_matrix[(30, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(30, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(30, 195)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_9(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 150)];

    groebner_matrix[(30, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(30, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(30, 194)] -= reduction_term;
    let factor = groebner_matrix[(30, 151)];
    groebner_matrix[(30, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(30, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(30, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(30, 196)] -= reduction_term;
    let factor = -groebner_matrix[(30, 154)];
    groebner_matrix[(30, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(30, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(30, 193)] -= reduction_term;
    let factor = groebner_matrix[(30, 163)];
    groebner_matrix[(30, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(30, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(30, 180)] -= reduction_term;
    let factor = groebner_matrix[(30, 164)];
    groebner_matrix[(30, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(30, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(30, 192)] -= reduction_term;
    let factor = -groebner_matrix[(30, 165)];
    groebner_matrix[(30, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(30, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(30, 191)] -= reduction_term;
    let factor = groebner_matrix[(30, 166)];
    groebner_matrix[(30, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(30, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(30, 183)] -= reduction_term;
    let factor = -groebner_matrix[(30, 167)];
    groebner_matrix[(30, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(30, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(30, 189)] -= reduction_term;
    let factor = groebner_matrix[(30, 168)];
    groebner_matrix[(30, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(30, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(30, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_10(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 169)];

    groebner_matrix[(30, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(30, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(30, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(30, 196)] -= reduction_term;
    let factor = groebner_matrix[(30, 172)];
    groebner_matrix[(30, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(30, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(30, 190)] -= reduction_term;
    let factor = -groebner_matrix[(30, 175)];
    groebner_matrix[(30, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(30, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(30, 187)] -= reduction_term;
    s_polynomial31(groebner_matrix);
    let factor = -groebner_matrix[(31, 1)];
    groebner_matrix[(31, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(31, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(31, 179)] -= reduction_term;
    let factor = groebner_matrix[(31, 2)];
    groebner_matrix[(31, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(31, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(31, 24)] -= reduction_term;
    let factor = -groebner_matrix[(31, 3)];
    groebner_matrix[(31, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(31, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(31, 180)] -= reduction_term;
    let factor = groebner_matrix[(31, 4)];
    groebner_matrix[(31, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(31, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(31, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(31, 190)] -= reduction_term;
    let factor = groebner_matrix[(31, 5)];
    groebner_matrix[(31, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(31, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(31, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(31, 188)] -= reduction_term;
    let factor = groebner_matrix[(31, 6)];
    groebner_matrix[(31, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(31, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(31, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(31, 189)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_11(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(31, 10)];
    groebner_matrix[(31, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 163)] -= reduction_term;
    let factor = -groebner_matrix[(31, 11)];
    groebner_matrix[(31, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 164)] -= reduction_term;
    let factor = -groebner_matrix[(31, 12)];
    groebner_matrix[(31, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 165)] -= reduction_term;
    let factor = groebner_matrix[(31, 13)];
    groebner_matrix[(31, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(31, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(31, 30)] -= reduction_term;
    let factor = -groebner_matrix[(31, 14)];
    groebner_matrix[(31, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(31, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(31, 182)] -= reduction_term;
    let factor = -groebner_matrix[(31, 15)];
    groebner_matrix[(31, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 166)] -= reduction_term;
    let factor = -groebner_matrix[(31, 18)];
    groebner_matrix[(31, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 167)] -= reduction_term;
    let factor = groebner_matrix[(31, 19)];
    groebner_matrix[(31, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(31, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(31, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(31, 192)] -= reduction_term;
    let factor = groebner_matrix[(31, 22)];
    groebner_matrix[(31, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(31, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(31, 123)] -= reduction_term;
    let factor = groebner_matrix[(31, 25)];
    groebner_matrix[(31, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(31, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(31, 35)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_12(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(31, 26)];
    groebner_matrix[(31, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(31, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(31, 183)] -= reduction_term;
    let factor = groebner_matrix[(31, 27)];
    groebner_matrix[(31, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(31, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(31, 175)] -= reduction_term;
    let factor = -groebner_matrix[(31, 30)];
    groebner_matrix[(31, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 168)] -= reduction_term;
    let factor = groebner_matrix[(31, 39)];
    groebner_matrix[(31, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(31, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(31, 101)] -= reduction_term;
    let factor = groebner_matrix[(31, 40)];
    groebner_matrix[(31, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(31, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(31, 158)] -= reduction_term;
    let factor = groebner_matrix[(31, 41)];
    groebner_matrix[(31, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(31, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(31, 102)] -= reduction_term;
    let factor = groebner_matrix[(31, 42)];
    groebner_matrix[(31, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(31, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(31, 159)] -= reduction_term;
    let factor = groebner_matrix[(31, 43)];
    groebner_matrix[(31, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(31, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(31, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(31, 193)] -= reduction_term;
    let factor = groebner_matrix[(31, 44)];
    groebner_matrix[(31, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(31, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(31, 113)] -= reduction_term;
    let factor = groebner_matrix[(31, 45)];
    groebner_matrix[(31, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(31, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(31, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_13(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(31, 46)];
    groebner_matrix[(31, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(31, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(31, 109)] -= reduction_term;
    let factor = groebner_matrix[(31, 47)];
    groebner_matrix[(31, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(31, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(31, 161)] -= reduction_term;
    let factor = -groebner_matrix[(31, 48)];
    groebner_matrix[(31, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 169)] -= reduction_term;
    let factor = groebner_matrix[(31, 49)];
    groebner_matrix[(31, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(31, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(31, 116)] -= reduction_term;
    let factor = -groebner_matrix[(31, 50)];
    groebner_matrix[(31, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(31, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(31, 154)] -= reduction_term;
    let factor = groebner_matrix[(31, 51)];
    groebner_matrix[(31, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(31, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(31, 114)] -= reduction_term;
    let factor = groebner_matrix[(31, 52)];
    groebner_matrix[(31, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(31, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(31, 162)] -= reduction_term;
    let factor = groebner_matrix[(31, 53)];
    groebner_matrix[(31, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(31, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(31, 146)] -= reduction_term;
    let factor = groebner_matrix[(31, 54)];
    groebner_matrix[(31, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(31, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(31, 117)] -= reduction_term;
    let factor = -groebner_matrix[(31, 55)];
    groebner_matrix[(31, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(31, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(31, 159)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_14(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(31, 56)];

    groebner_matrix[(31, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(31, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(31, 158)] -= reduction_term;
    let factor = groebner_matrix[(31, 57)];
    groebner_matrix[(31, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(31, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(31, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(31, 188)] -= reduction_term;
    let factor = groebner_matrix[(31, 58)];
    groebner_matrix[(31, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(31, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(31, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(31, 189)] -= reduction_term;
    let factor = groebner_matrix[(31, 59)];
    groebner_matrix[(31, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(31, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(31, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(31, 191)] -= reduction_term;
    let factor = groebner_matrix[(31, 60)];
    groebner_matrix[(31, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(31, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(31, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(31, 192)] -= reduction_term;
    let factor = groebner_matrix[(31, 61)];
    groebner_matrix[(31, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(31, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(31, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(31, 194)] -= reduction_term;
    let factor = groebner_matrix[(31, 64)];
    groebner_matrix[(31, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 148)] -= reduction_term;
    let factor = groebner_matrix[(31, 65)];
    groebner_matrix[(31, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 149)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_15(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(31, 66)];

    groebner_matrix[(31, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 150)] -= reduction_term;
    let factor = groebner_matrix[(31, 67)];
    groebner_matrix[(31, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(31, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(31, 78)] -= reduction_term;
    let factor = -groebner_matrix[(31, 68)];
    groebner_matrix[(31, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(31, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(31, 185)] -= reduction_term;
    let factor = groebner_matrix[(31, 69)];
    groebner_matrix[(31, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 151)] -= reduction_term;
    let factor = groebner_matrix[(31, 70)];
    groebner_matrix[(31, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(31, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(31, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(31, 194)] -= reduction_term;
    let factor = -groebner_matrix[(31, 73)];
    groebner_matrix[(31, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 176)] -= reduction_term;
    let factor = -groebner_matrix[(31, 76)];
    groebner_matrix[(31, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 142)] -= reduction_term;
    let factor = -groebner_matrix[(31, 77)];
    groebner_matrix[(31, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 143)] -= reduction_term;
    let factor = groebner_matrix[(31, 78)];
    groebner_matrix[(31, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 160)] -= reduction_term;
    let factor = -groebner_matrix[(31, 79)];
    groebner_matrix[(31, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 148)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_16(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(31, 80)];

    groebner_matrix[(31, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 152)] -= reduction_term;
    let factor = -groebner_matrix[(31, 81)];
    groebner_matrix[(31, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 157)] -= reduction_term;
    let factor = groebner_matrix[(31, 82)];
    groebner_matrix[(31, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(31, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(31, 127)] -= reduction_term;
    let factor = groebner_matrix[(31, 83)];
    groebner_matrix[(31, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(31, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(31, 175)] -= reduction_term;
    let factor = groebner_matrix[(31, 84)];
    groebner_matrix[(31, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 166)] -= reduction_term;
    let factor = groebner_matrix[(31, 85)];
    groebner_matrix[(31, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(31, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(31, 130)] -= reduction_term;
    let factor = -groebner_matrix[(31, 86)];
    groebner_matrix[(31, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(31, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(31, 172)] -= reduction_term;
    let factor = -groebner_matrix[(31, 87)];
    groebner_matrix[(31, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 163)] -= reduction_term;
    let factor = groebner_matrix[(31, 91)];
    groebner_matrix[(31, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 173)] -= reduction_term;
    let factor = -groebner_matrix[(31, 94)];
    groebner_matrix[(31, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 170)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_17(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(31, 97)];

    groebner_matrix[(31, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(31, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(31, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(31, 195)] -= reduction_term;
    let factor = groebner_matrix[(31, 100)];
    groebner_matrix[(31, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(31, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(31, 139)] -= reduction_term;
    let factor = groebner_matrix[(31, 103)];
    groebner_matrix[(31, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(31, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(31, 114)] -= reduction_term;
    let factor = -groebner_matrix[(31, 104)];
    groebner_matrix[(31, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(31, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(31, 186)] -= reduction_term;
    let factor = groebner_matrix[(31, 105)];
    groebner_matrix[(31, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(31, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(31, 185)] -= reduction_term;
    let factor = groebner_matrix[(31, 106)];
    groebner_matrix[(31, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(31, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(31, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(31, 195)] -= reduction_term;
    let factor = -groebner_matrix[(31, 109)];
    groebner_matrix[(31, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 184)] -= reduction_term;
    let factor = groebner_matrix[(31, 118)];
    groebner_matrix[(31, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(31, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(31, 135)] -= reduction_term;
    let factor = groebner_matrix[(31, 119)];
    groebner_matrix[(31, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(31, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(31, 183)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_18(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(31, 120)];

    groebner_matrix[(31, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(31, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(31, 182)] -= reduction_term;
    let factor = groebner_matrix[(31, 121)];
    groebner_matrix[(31, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(31, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(31, 138)] -= reduction_term;
    let factor = -groebner_matrix[(31, 122)];
    groebner_matrix[(31, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(31, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(31, 180)] -= reduction_term;
    let factor = groebner_matrix[(31, 123)];
    groebner_matrix[(31, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(31, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(31, 179)] -= reduction_term;
    let factor = groebner_matrix[(31, 127)];
    groebner_matrix[(31, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 181)] -= reduction_term;
    let factor = -groebner_matrix[(31, 130)];
    groebner_matrix[(31, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 178)] -= reduction_term;
    let factor = groebner_matrix[(31, 142)];
    groebner_matrix[(31, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(31, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(31, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(31, 196)] -= reduction_term;
    let factor = groebner_matrix[(31, 143)];
    groebner_matrix[(31, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(31, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(31, 176)] -= reduction_term;
    let factor = groebner_matrix[(31, 144)];
    groebner_matrix[(31, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(31, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(31, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(31, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_19(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(31, 146)];

    groebner_matrix[(31, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(31, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(31, 185)] -= reduction_term;
    let factor = groebner_matrix[(31, 147)];
    groebner_matrix[(31, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(31, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(31, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(31, 196)] -= reduction_term;
    let factor = groebner_matrix[(31, 148)];
    groebner_matrix[(31, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(31, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(31, 159)] -= reduction_term;
    let factor = -groebner_matrix[(31, 149)];
    groebner_matrix[(31, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(31, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(31, 195)] -= reduction_term;
    let factor = groebner_matrix[(31, 150)];
    groebner_matrix[(31, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(31, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(31, 194)] -= reduction_term;
    let factor = groebner_matrix[(31, 151)];
    groebner_matrix[(31, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(31, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(31, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(31, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 31);
    let factor = -groebner_matrix[(31, 154)];
    groebner_matrix[(31, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(31, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(31, 193)] -= reduction_term;
    let factor = groebner_matrix[(31, 163)];
    groebner_matrix[(31, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(31, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(31, 180)] -= reduction_term;
    let factor = groebner_matrix[(31, 164)];
    groebner_matrix[(31, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(31, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(31, 192)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_20(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(31, 165)];

    groebner_matrix[(31, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(31, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(31, 191)] -= reduction_term;
    let factor = groebner_matrix[(31, 166)];
    groebner_matrix[(31, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(31, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(31, 183)] -= reduction_term;
    let factor = -groebner_matrix[(31, 167)];
    groebner_matrix[(31, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(31, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(31, 189)] -= reduction_term;
    let factor = groebner_matrix[(31, 168)];
    groebner_matrix[(31, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(31, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(31, 188)] -= reduction_term;
    let factor = groebner_matrix[(31, 169)];
    groebner_matrix[(31, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(31, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(31, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(31, 196)] -= reduction_term;
    let factor = groebner_matrix[(31, 172)];
    groebner_matrix[(31, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(31, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(31, 190)] -= reduction_term;
    let factor = -groebner_matrix[(31, 175)];
    groebner_matrix[(31, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(31, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(31, 187)] -= reduction_term;
    s_polynomial32(groebner_matrix);
    let factor = -groebner_matrix[(32, 1)];
    groebner_matrix[(32, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(32, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(32, 179)] -= reduction_term;
    let factor = groebner_matrix[(32, 2)];
    groebner_matrix[(32, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(32, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(32, 24)] -= reduction_term;
    let factor = -groebner_matrix[(32, 3)];
    groebner_matrix[(32, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(32, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(32, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_21(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(32, 4)];
    groebner_matrix[(32, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(32, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(32, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(32, 190)] -= reduction_term;
    let factor = groebner_matrix[(32, 5)];
    groebner_matrix[(32, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(32, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(32, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(32, 188)] -= reduction_term;
    let factor = groebner_matrix[(32, 6)];
    groebner_matrix[(32, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(32, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(32, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(32, 189)] -= reduction_term;
    let factor = -groebner_matrix[(32, 10)];
    groebner_matrix[(32, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 163)] -= reduction_term;
    let factor = -groebner_matrix[(32, 11)];
    groebner_matrix[(32, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 164)] -= reduction_term;
    let factor = -groebner_matrix[(32, 12)];
    groebner_matrix[(32, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 165)] -= reduction_term;
    let factor = groebner_matrix[(32, 13)];
    groebner_matrix[(32, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(32, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(32, 30)] -= reduction_term;
    let factor = -groebner_matrix[(32, 14)];
    groebner_matrix[(32, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(32, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(32, 182)] -= reduction_term;
    let factor = -groebner_matrix[(32, 15)];
    groebner_matrix[(32, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 166)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_22(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(32, 18)];

    groebner_matrix[(32, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 167)] -= reduction_term;
    let factor = groebner_matrix[(32, 19)];
    groebner_matrix[(32, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(32, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(32, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(32, 192)] -= reduction_term;
    let factor = groebner_matrix[(32, 22)];
    groebner_matrix[(32, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(32, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(32, 123)] -= reduction_term;
    let factor = groebner_matrix[(32, 25)];
    groebner_matrix[(32, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(32, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(32, 35)] -= reduction_term;
    let factor = -groebner_matrix[(32, 26)];
    groebner_matrix[(32, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(32, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(32, 183)] -= reduction_term;
    let factor = groebner_matrix[(32, 27)];
    groebner_matrix[(32, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(32, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(32, 175)] -= reduction_term;
    let factor = -groebner_matrix[(32, 30)];
    groebner_matrix[(32, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 168)] -= reduction_term;
    let factor = groebner_matrix[(32, 39)];
    groebner_matrix[(32, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(32, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(32, 101)] -= reduction_term;
    let factor = groebner_matrix[(32, 40)];
    groebner_matrix[(32, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(32, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(32, 158)] -= reduction_term;
    let factor = groebner_matrix[(32, 41)];
    groebner_matrix[(32, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(32, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(32, 102)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_23(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(32, 42)];

    groebner_matrix[(32, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(32, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(32, 159)] -= reduction_term;
    let factor = groebner_matrix[(32, 43)];
    groebner_matrix[(32, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(32, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(32, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(32, 193)] -= reduction_term;
    let factor = groebner_matrix[(32, 44)];
    groebner_matrix[(32, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(32, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(32, 113)] -= reduction_term;
    let factor = groebner_matrix[(32, 45)];
    groebner_matrix[(32, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(32, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(32, 114)] -= reduction_term;
    let factor = groebner_matrix[(32, 46)];
    groebner_matrix[(32, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(32, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(32, 109)] -= reduction_term;
    let factor = groebner_matrix[(32, 47)];
    groebner_matrix[(32, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(32, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(32, 161)] -= reduction_term;
    let factor = -groebner_matrix[(32, 48)];
    groebner_matrix[(32, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 169)] -= reduction_term;
    let factor = groebner_matrix[(32, 49)];
    groebner_matrix[(32, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(32, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(32, 116)] -= reduction_term;
    let factor = -groebner_matrix[(32, 50)];
    groebner_matrix[(32, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(32, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(32, 154)] -= reduction_term;
    let factor = groebner_matrix[(32, 51)];
    groebner_matrix[(32, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(32, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(32, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_24(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(32, 52)];

    groebner_matrix[(32, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(32, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(32, 162)] -= reduction_term;
    let factor = groebner_matrix[(32, 53)];
    groebner_matrix[(32, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(32, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(32, 146)] -= reduction_term;
    let factor = groebner_matrix[(32, 54)];
    groebner_matrix[(32, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(32, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(32, 117)] -= reduction_term;
    let factor = -groebner_matrix[(32, 55)];
    groebner_matrix[(32, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(32, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(32, 159)] -= reduction_term;
    let factor = groebner_matrix[(32, 56)];
    groebner_matrix[(32, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(32, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(32, 158)] -= reduction_term;
    let factor = groebner_matrix[(32, 57)];
    groebner_matrix[(32, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(32, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(32, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(32, 188)] -= reduction_term;
    let factor = groebner_matrix[(32, 58)];
    groebner_matrix[(32, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(32, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(32, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(32, 189)] -= reduction_term;
    let factor = groebner_matrix[(32, 59)];
    groebner_matrix[(32, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(32, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(32, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(32, 191)] -= reduction_term;
    let factor = groebner_matrix[(32, 60)];
    groebner_matrix[(32, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(32, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(32, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(32, 192)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_25(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(32, 61)];

    groebner_matrix[(32, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(32, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(32, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(32, 194)] -= reduction_term;
    let factor = groebner_matrix[(32, 64)];
    groebner_matrix[(32, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 148)] -= reduction_term;
    let factor = groebner_matrix[(32, 65)];
    groebner_matrix[(32, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 149)] -= reduction_term;
    let factor = groebner_matrix[(32, 66)];
    groebner_matrix[(32, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 150)] -= reduction_term;
    let factor = groebner_matrix[(32, 67)];
    groebner_matrix[(32, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(32, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(32, 78)] -= reduction_term;
    let factor = -groebner_matrix[(32, 68)];
    groebner_matrix[(32, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(32, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(32, 185)] -= reduction_term;
    let factor = groebner_matrix[(32, 69)];
    groebner_matrix[(32, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 151)] -= reduction_term;
    let factor = groebner_matrix[(32, 70)];
    groebner_matrix[(32, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(32, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(32, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(32, 194)] -= reduction_term;
    let factor = -groebner_matrix[(32, 73)];
    groebner_matrix[(32, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 176)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_26(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(32, 76)];

    groebner_matrix[(32, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 142)] -= reduction_term;
    let factor = -groebner_matrix[(32, 77)];
    groebner_matrix[(32, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 143)] -= reduction_term;
    let factor = groebner_matrix[(32, 78)];
    groebner_matrix[(32, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 160)] -= reduction_term;
    let factor = -groebner_matrix[(32, 79)];
    groebner_matrix[(32, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 148)] -= reduction_term;
    let factor = -groebner_matrix[(32, 80)];
    groebner_matrix[(32, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 152)] -= reduction_term;
    let factor = -groebner_matrix[(32, 81)];
    groebner_matrix[(32, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 157)] -= reduction_term;
    let factor = groebner_matrix[(32, 82)];
    groebner_matrix[(32, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(32, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(32, 127)] -= reduction_term;
    let factor = groebner_matrix[(32, 83)];
    groebner_matrix[(32, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(32, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(32, 175)] -= reduction_term;
    let factor = groebner_matrix[(32, 84)];
    groebner_matrix[(32, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 166)] -= reduction_term;
    let factor = groebner_matrix[(32, 85)];
    groebner_matrix[(32, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(32, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(32, 130)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_27(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(32, 86)];

    groebner_matrix[(32, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(32, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(32, 172)] -= reduction_term;
    let factor = -groebner_matrix[(32, 87)];
    groebner_matrix[(32, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 163)] -= reduction_term;
    let factor = groebner_matrix[(32, 91)];
    groebner_matrix[(32, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 173)] -= reduction_term;
    let factor = -groebner_matrix[(32, 94)];
    groebner_matrix[(32, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 170)] -= reduction_term;
    let factor = groebner_matrix[(32, 97)];
    groebner_matrix[(32, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(32, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(32, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(32, 195)] -= reduction_term;
    let factor = groebner_matrix[(32, 100)];
    groebner_matrix[(32, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(32, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(32, 139)] -= reduction_term;
    let factor = groebner_matrix[(32, 103)];
    groebner_matrix[(32, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(32, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(32, 114)] -= reduction_term;
    let factor = -groebner_matrix[(32, 104)];
    groebner_matrix[(32, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(32, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(32, 186)] -= reduction_term;
    let factor = groebner_matrix[(32, 105)];
    groebner_matrix[(32, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(32, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(32, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_28(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(32, 106)];

    groebner_matrix[(32, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(32, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(32, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(32, 195)] -= reduction_term;
    let factor = -groebner_matrix[(32, 109)];
    groebner_matrix[(32, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 184)] -= reduction_term;
    let factor = groebner_matrix[(32, 118)];
    groebner_matrix[(32, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(32, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(32, 135)] -= reduction_term;
    let factor = groebner_matrix[(32, 119)];
    groebner_matrix[(32, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(32, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(32, 183)] -= reduction_term;
    let factor = -groebner_matrix[(32, 120)];
    groebner_matrix[(32, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(32, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(32, 182)] -= reduction_term;
    let factor = groebner_matrix[(32, 121)];
    groebner_matrix[(32, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(32, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(32, 138)] -= reduction_term;
    let factor = -groebner_matrix[(32, 122)];
    groebner_matrix[(32, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(32, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(32, 180)] -= reduction_term;
    let factor = groebner_matrix[(32, 123)];
    groebner_matrix[(32, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(32, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(32, 179)] -= reduction_term;
    let factor = groebner_matrix[(32, 127)];
    groebner_matrix[(32, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 181)] -= reduction_term;
    let factor = -groebner_matrix[(32, 130)];
    groebner_matrix[(32, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 178)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_29(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(32, 142)];

    groebner_matrix[(32, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(32, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(32, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(32, 196)] -= reduction_term;
    let factor = groebner_matrix[(32, 143)];
    groebner_matrix[(32, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(32, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(32, 176)] -= reduction_term;
    let factor = groebner_matrix[(32, 144)];
    groebner_matrix[(32, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(32, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(32, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(32, 196)] -= reduction_term;
    let factor = groebner_matrix[(32, 146)];
    groebner_matrix[(32, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(32, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(32, 185)] -= reduction_term;
    let factor = groebner_matrix[(32, 147)];
    groebner_matrix[(32, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(32, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(32, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(32, 196)] -= reduction_term;
    let factor = groebner_matrix[(32, 148)];
    groebner_matrix[(32, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(32, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(32, 159)] -= reduction_term;
    let factor = -groebner_matrix[(32, 149)];
    groebner_matrix[(32, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(32, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(32, 195)] -= reduction_term;
    let factor = groebner_matrix[(32, 150)];
    groebner_matrix[(32, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(32, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(32, 194)] -= reduction_term;
    let factor = groebner_matrix[(32, 151)];
    groebner_matrix[(32, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(32, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(32, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(32, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 32);
}

#[inline]
fn compute_basis_stage_30(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row31_000000000_f(groebner_matrix, 32);
    let factor = -groebner_matrix[(32, 154)];
    groebner_matrix[(32, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(32, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(32, 193)] -= reduction_term;
    let factor = groebner_matrix[(32, 163)];
    groebner_matrix[(32, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(32, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(32, 180)] -= reduction_term;
    let factor = groebner_matrix[(32, 164)];
    groebner_matrix[(32, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(32, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(32, 192)] -= reduction_term;
    let factor = -groebner_matrix[(32, 165)];
    groebner_matrix[(32, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(32, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(32, 191)] -= reduction_term;
    let factor = groebner_matrix[(32, 166)];
    groebner_matrix[(32, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(32, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(32, 183)] -= reduction_term;
    let factor = -groebner_matrix[(32, 167)];
    groebner_matrix[(32, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(32, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(32, 189)] -= reduction_term;
    let factor = groebner_matrix[(32, 168)];
    groebner_matrix[(32, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(32, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(32, 188)] -= reduction_term;
    let factor = groebner_matrix[(32, 169)];
    groebner_matrix[(32, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(32, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(32, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(32, 196)] -= reduction_term;
    let factor = groebner_matrix[(32, 172)];
    groebner_matrix[(32, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(32, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(32, 190)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_31(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(32, 175)];

    groebner_matrix[(32, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(32, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(32, 187)] -= reduction_term;
    s_polynomial33(groebner_matrix);
    let factor = -groebner_matrix[(33, 1)];
    groebner_matrix[(33, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(33, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(33, 179)] -= reduction_term;
    let factor = groebner_matrix[(33, 2)];
    groebner_matrix[(33, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(33, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(33, 24)] -= reduction_term;
    let factor = -groebner_matrix[(33, 3)];
    groebner_matrix[(33, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(33, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(33, 180)] -= reduction_term;
    let factor = groebner_matrix[(33, 4)];
    groebner_matrix[(33, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(33, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(33, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(33, 190)] -= reduction_term;
    let factor = groebner_matrix[(33, 5)];
    groebner_matrix[(33, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(33, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(33, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(33, 188)] -= reduction_term;
    let factor = groebner_matrix[(33, 6)];
    groebner_matrix[(33, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(33, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(33, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(33, 189)] -= reduction_term;
    let factor = -groebner_matrix[(33, 10)];
    groebner_matrix[(33, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 163)] -= reduction_term;
    let factor = -groebner_matrix[(33, 11)];
    groebner_matrix[(33, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 164)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_32(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(33, 12)];

    groebner_matrix[(33, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 165)] -= reduction_term;
    let factor = groebner_matrix[(33, 13)];
    groebner_matrix[(33, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(33, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(33, 30)] -= reduction_term;
    let factor = -groebner_matrix[(33, 14)];
    groebner_matrix[(33, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(33, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(33, 182)] -= reduction_term;
    let factor = -groebner_matrix[(33, 15)];
    groebner_matrix[(33, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 166)] -= reduction_term;
    let factor = -groebner_matrix[(33, 18)];
    groebner_matrix[(33, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 167)] -= reduction_term;
    let factor = groebner_matrix[(33, 19)];
    groebner_matrix[(33, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(33, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(33, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(33, 192)] -= reduction_term;
    let factor = groebner_matrix[(33, 22)];
    groebner_matrix[(33, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(33, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(33, 123)] -= reduction_term;
    let factor = groebner_matrix[(33, 25)];
    groebner_matrix[(33, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(33, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(33, 35)] -= reduction_term;
    let factor = -groebner_matrix[(33, 26)];
    groebner_matrix[(33, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(33, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(33, 183)] -= reduction_term;
    let factor = groebner_matrix[(33, 27)];
    groebner_matrix[(33, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(33, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(33, 175)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_33(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(33, 30)];

    groebner_matrix[(33, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 168)] -= reduction_term;
    let factor = groebner_matrix[(33, 39)];
    groebner_matrix[(33, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(33, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(33, 101)] -= reduction_term;
    let factor = groebner_matrix[(33, 40)];
    groebner_matrix[(33, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(33, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(33, 158)] -= reduction_term;
    let factor = groebner_matrix[(33, 41)];
    groebner_matrix[(33, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(33, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(33, 102)] -= reduction_term;
    let factor = groebner_matrix[(33, 42)];
    groebner_matrix[(33, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(33, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(33, 159)] -= reduction_term;
    let factor = groebner_matrix[(33, 43)];
    groebner_matrix[(33, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(33, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(33, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(33, 193)] -= reduction_term;
    let factor = groebner_matrix[(33, 44)];
    groebner_matrix[(33, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(33, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(33, 113)] -= reduction_term;
    let factor = groebner_matrix[(33, 45)];
    groebner_matrix[(33, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(33, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(33, 114)] -= reduction_term;
    let factor = groebner_matrix[(33, 46)];
    groebner_matrix[(33, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(33, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(33, 109)] -= reduction_term;
    let factor = groebner_matrix[(33, 47)];
    groebner_matrix[(33, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(33, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(33, 161)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_34(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(33, 48)];

    groebner_matrix[(33, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 169)] -= reduction_term;
    let factor = groebner_matrix[(33, 49)];
    groebner_matrix[(33, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(33, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(33, 116)] -= reduction_term;
    let factor = -groebner_matrix[(33, 50)];
    groebner_matrix[(33, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(33, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(33, 154)] -= reduction_term;
    let factor = groebner_matrix[(33, 51)];
    groebner_matrix[(33, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(33, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(33, 114)] -= reduction_term;
    let factor = groebner_matrix[(33, 52)];
    groebner_matrix[(33, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(33, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(33, 162)] -= reduction_term;
    let factor = groebner_matrix[(33, 53)];
    groebner_matrix[(33, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(33, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(33, 146)] -= reduction_term;
    let factor = groebner_matrix[(33, 54)];
    groebner_matrix[(33, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(33, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(33, 117)] -= reduction_term;
    let factor = -groebner_matrix[(33, 55)];
    groebner_matrix[(33, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(33, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(33, 159)] -= reduction_term;
    let factor = groebner_matrix[(33, 56)];
    groebner_matrix[(33, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(33, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(33, 158)] -= reduction_term;
    let factor = groebner_matrix[(33, 57)];
    groebner_matrix[(33, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(33, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(33, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(33, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_35(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(33, 58)];

    groebner_matrix[(33, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(33, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(33, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(33, 189)] -= reduction_term;
    let factor = groebner_matrix[(33, 59)];
    groebner_matrix[(33, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(33, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(33, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(33, 191)] -= reduction_term;
    let factor = groebner_matrix[(33, 60)];
    groebner_matrix[(33, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(33, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(33, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(33, 192)] -= reduction_term;
    let factor = groebner_matrix[(33, 61)];
    groebner_matrix[(33, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(33, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(33, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(33, 194)] -= reduction_term;
    let factor = groebner_matrix[(33, 64)];
    groebner_matrix[(33, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 148)] -= reduction_term;
    let factor = groebner_matrix[(33, 65)];
    groebner_matrix[(33, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 149)] -= reduction_term;
    let factor = groebner_matrix[(33, 66)];
    groebner_matrix[(33, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 150)] -= reduction_term;
    let factor = groebner_matrix[(33, 67)];
    groebner_matrix[(33, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(33, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(33, 78)] -= reduction_term;
    let factor = -groebner_matrix[(33, 68)];
    groebner_matrix[(33, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(33, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(33, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_36(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(33, 69)];

    groebner_matrix[(33, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 151)] -= reduction_term;
    let factor = groebner_matrix[(33, 70)];
    groebner_matrix[(33, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(33, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(33, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(33, 194)] -= reduction_term;
    let factor = -groebner_matrix[(33, 73)];
    groebner_matrix[(33, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 176)] -= reduction_term;
    let factor = -groebner_matrix[(33, 76)];
    groebner_matrix[(33, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 142)] -= reduction_term;
    let factor = -groebner_matrix[(33, 77)];
    groebner_matrix[(33, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 143)] -= reduction_term;
    let factor = groebner_matrix[(33, 78)];
    groebner_matrix[(33, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 160)] -= reduction_term;
    let factor = -groebner_matrix[(33, 79)];
    groebner_matrix[(33, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 148)] -= reduction_term;
    let factor = -groebner_matrix[(33, 80)];
    groebner_matrix[(33, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 152)] -= reduction_term;
    let factor = -groebner_matrix[(33, 81)];
    groebner_matrix[(33, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 157)] -= reduction_term;
    let factor = groebner_matrix[(33, 82)];
    groebner_matrix[(33, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(33, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(33, 127)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_37(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(33, 83)];

    groebner_matrix[(33, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(33, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(33, 175)] -= reduction_term;
    let factor = groebner_matrix[(33, 84)];
    groebner_matrix[(33, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 166)] -= reduction_term;
    let factor = groebner_matrix[(33, 85)];
    groebner_matrix[(33, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(33, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(33, 130)] -= reduction_term;
    let factor = -groebner_matrix[(33, 86)];
    groebner_matrix[(33, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(33, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(33, 172)] -= reduction_term;
    let factor = -groebner_matrix[(33, 87)];
    groebner_matrix[(33, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 163)] -= reduction_term;
    let factor = groebner_matrix[(33, 91)];
    groebner_matrix[(33, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 173)] -= reduction_term;
    let factor = -groebner_matrix[(33, 94)];
    groebner_matrix[(33, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 170)] -= reduction_term;
    let factor = groebner_matrix[(33, 97)];
    groebner_matrix[(33, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(33, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(33, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(33, 195)] -= reduction_term;
    let factor = groebner_matrix[(33, 100)];
    groebner_matrix[(33, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(33, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(33, 139)] -= reduction_term;
    let factor = groebner_matrix[(33, 103)];
    groebner_matrix[(33, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(33, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(33, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_38(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(33, 104)];

    groebner_matrix[(33, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(33, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(33, 186)] -= reduction_term;
    let factor = groebner_matrix[(33, 105)];
    groebner_matrix[(33, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(33, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(33, 185)] -= reduction_term;
    let factor = groebner_matrix[(33, 106)];
    groebner_matrix[(33, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(33, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(33, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(33, 195)] -= reduction_term;
    let factor = -groebner_matrix[(33, 109)];
    groebner_matrix[(33, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 184)] -= reduction_term;
    let factor = groebner_matrix[(33, 118)];
    groebner_matrix[(33, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(33, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(33, 135)] -= reduction_term;
    let factor = groebner_matrix[(33, 119)];
    groebner_matrix[(33, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(33, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(33, 183)] -= reduction_term;
    let factor = -groebner_matrix[(33, 120)];
    groebner_matrix[(33, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(33, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(33, 182)] -= reduction_term;
    let factor = groebner_matrix[(33, 121)];
    groebner_matrix[(33, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(33, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(33, 138)] -= reduction_term;
    let factor = -groebner_matrix[(33, 122)];
    groebner_matrix[(33, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(33, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(33, 180)] -= reduction_term;
    let factor = groebner_matrix[(33, 123)];
    groebner_matrix[(33, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(33, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(33, 179)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_39(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(33, 127)];

    groebner_matrix[(33, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 181)] -= reduction_term;
    let factor = -groebner_matrix[(33, 130)];
    groebner_matrix[(33, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 178)] -= reduction_term;
    let factor = groebner_matrix[(33, 142)];
    groebner_matrix[(33, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(33, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(33, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(33, 196)] -= reduction_term;
    let factor = groebner_matrix[(33, 143)];
    groebner_matrix[(33, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(33, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(33, 176)] -= reduction_term;
    let factor = groebner_matrix[(33, 144)];
    groebner_matrix[(33, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(33, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(33, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(33, 196)] -= reduction_term;
    let factor = groebner_matrix[(33, 146)];
    groebner_matrix[(33, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(33, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(33, 185)] -= reduction_term;
    let factor = groebner_matrix[(33, 147)];
    groebner_matrix[(33, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(33, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(33, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(33, 196)] -= reduction_term;
    let factor = groebner_matrix[(33, 148)];
    groebner_matrix[(33, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(33, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(33, 159)] -= reduction_term;
    let factor = -groebner_matrix[(33, 149)];
    groebner_matrix[(33, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(33, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(33, 195)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_40(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(33, 150)];

    groebner_matrix[(33, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(33, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(33, 194)] -= reduction_term;
    let factor = groebner_matrix[(33, 151)];
    groebner_matrix[(33, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(33, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(33, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(33, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 33);
    groebner_row31_000000000_f(groebner_matrix, 33);
    let factor = -groebner_matrix[(33, 154)];
    groebner_matrix[(33, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(33, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(33, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 33);
    let factor = groebner_matrix[(33, 163)];
    groebner_matrix[(33, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(33, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(33, 180)] -= reduction_term;
    let factor = groebner_matrix[(33, 164)];
    groebner_matrix[(33, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(33, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(33, 192)] -= reduction_term;
    let factor = -groebner_matrix[(33, 165)];
    groebner_matrix[(33, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(33, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(33, 191)] -= reduction_term;
    let factor = groebner_matrix[(33, 166)];
    groebner_matrix[(33, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(33, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(33, 183)] -= reduction_term;
    let factor = -groebner_matrix[(33, 167)];
    groebner_matrix[(33, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(33, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(33, 189)] -= reduction_term;
    let factor = groebner_matrix[(33, 168)];
    groebner_matrix[(33, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(33, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(33, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_41(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(33, 169)];

    groebner_matrix[(33, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(33, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(33, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(33, 196)] -= reduction_term;
    let factor = groebner_matrix[(33, 172)];
    groebner_matrix[(33, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(33, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(33, 190)] -= reduction_term;
    let factor = -groebner_matrix[(33, 175)];
    groebner_matrix[(33, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(33, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(33, 187)] -= reduction_term;
    s_polynomial34(groebner_matrix);
    let factor = -groebner_matrix[(34, 1)];
    groebner_matrix[(34, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(34, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(34, 179)] -= reduction_term;
    let factor = groebner_matrix[(34, 2)];
    groebner_matrix[(34, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(34, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(34, 24)] -= reduction_term;
    let factor = -groebner_matrix[(34, 3)];
    groebner_matrix[(34, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(34, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(34, 180)] -= reduction_term;
    let factor = groebner_matrix[(34, 4)];
    groebner_matrix[(34, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(34, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(34, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(34, 190)] -= reduction_term;
    let factor = groebner_matrix[(34, 5)];
    groebner_matrix[(34, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(34, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(34, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(34, 188)] -= reduction_term;
    let factor = groebner_matrix[(34, 6)];
    groebner_matrix[(34, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(34, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(34, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(34, 189)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_42(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(34, 10)];
    groebner_matrix[(34, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 163)] -= reduction_term;
    let factor = -groebner_matrix[(34, 11)];
    groebner_matrix[(34, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 164)] -= reduction_term;
    let factor = -groebner_matrix[(34, 12)];
    groebner_matrix[(34, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 165)] -= reduction_term;
    let factor = groebner_matrix[(34, 13)];
    groebner_matrix[(34, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(34, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(34, 30)] -= reduction_term;
    let factor = -groebner_matrix[(34, 14)];
    groebner_matrix[(34, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(34, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(34, 182)] -= reduction_term;
    let factor = -groebner_matrix[(34, 15)];
    groebner_matrix[(34, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 166)] -= reduction_term;
    let factor = -groebner_matrix[(34, 18)];
    groebner_matrix[(34, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 167)] -= reduction_term;
    let factor = groebner_matrix[(34, 19)];
    groebner_matrix[(34, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(34, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(34, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(34, 192)] -= reduction_term;
    let factor = groebner_matrix[(34, 22)];
    groebner_matrix[(34, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(34, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(34, 123)] -= reduction_term;
    let factor = groebner_matrix[(34, 25)];
    groebner_matrix[(34, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(34, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(34, 35)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_43(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(34, 26)];
    groebner_matrix[(34, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(34, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(34, 183)] -= reduction_term;
    let factor = groebner_matrix[(34, 27)];
    groebner_matrix[(34, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(34, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(34, 175)] -= reduction_term;
    let factor = -groebner_matrix[(34, 30)];
    groebner_matrix[(34, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 168)] -= reduction_term;
    let factor = groebner_matrix[(34, 39)];
    groebner_matrix[(34, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(34, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(34, 101)] -= reduction_term;
    let factor = groebner_matrix[(34, 40)];
    groebner_matrix[(34, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(34, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(34, 158)] -= reduction_term;
    let factor = groebner_matrix[(34, 41)];
    groebner_matrix[(34, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(34, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(34, 102)] -= reduction_term;
    let factor = groebner_matrix[(34, 42)];
    groebner_matrix[(34, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(34, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(34, 159)] -= reduction_term;
    let factor = groebner_matrix[(34, 43)];
    groebner_matrix[(34, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(34, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(34, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(34, 193)] -= reduction_term;
    let factor = groebner_matrix[(34, 44)];
    groebner_matrix[(34, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(34, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(34, 113)] -= reduction_term;
    let factor = groebner_matrix[(34, 45)];
    groebner_matrix[(34, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(34, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(34, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_44(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(34, 46)];
    groebner_matrix[(34, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(34, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(34, 109)] -= reduction_term;
    let factor = groebner_matrix[(34, 47)];
    groebner_matrix[(34, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(34, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(34, 161)] -= reduction_term;
    let factor = -groebner_matrix[(34, 48)];
    groebner_matrix[(34, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 169)] -= reduction_term;
    let factor = groebner_matrix[(34, 49)];
    groebner_matrix[(34, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(34, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(34, 116)] -= reduction_term;
    let factor = -groebner_matrix[(34, 50)];
    groebner_matrix[(34, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(34, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(34, 154)] -= reduction_term;
    let factor = groebner_matrix[(34, 51)];
    groebner_matrix[(34, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(34, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(34, 114)] -= reduction_term;
    let factor = groebner_matrix[(34, 52)];
    groebner_matrix[(34, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(34, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(34, 162)] -= reduction_term;
    let factor = groebner_matrix[(34, 53)];
    groebner_matrix[(34, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(34, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(34, 146)] -= reduction_term;
    let factor = groebner_matrix[(34, 54)];
    groebner_matrix[(34, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(34, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(34, 117)] -= reduction_term;
    let factor = -groebner_matrix[(34, 55)];
    groebner_matrix[(34, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(34, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(34, 159)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_45(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(34, 56)];

    groebner_matrix[(34, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(34, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(34, 158)] -= reduction_term;
    let factor = groebner_matrix[(34, 57)];
    groebner_matrix[(34, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(34, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(34, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(34, 188)] -= reduction_term;
    let factor = groebner_matrix[(34, 58)];
    groebner_matrix[(34, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(34, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(34, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(34, 189)] -= reduction_term;
    let factor = groebner_matrix[(34, 59)];
    groebner_matrix[(34, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(34, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(34, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(34, 191)] -= reduction_term;
    let factor = groebner_matrix[(34, 60)];
    groebner_matrix[(34, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(34, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(34, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(34, 192)] -= reduction_term;
    let factor = groebner_matrix[(34, 61)];
    groebner_matrix[(34, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(34, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(34, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(34, 194)] -= reduction_term;
    let factor = groebner_matrix[(34, 64)];
    groebner_matrix[(34, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 148)] -= reduction_term;
    let factor = groebner_matrix[(34, 65)];
    groebner_matrix[(34, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 149)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_46(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(34, 66)];

    groebner_matrix[(34, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 150)] -= reduction_term;
    let factor = groebner_matrix[(34, 67)];
    groebner_matrix[(34, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(34, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(34, 78)] -= reduction_term;
    let factor = -groebner_matrix[(34, 68)];
    groebner_matrix[(34, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(34, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(34, 185)] -= reduction_term;
    let factor = groebner_matrix[(34, 69)];
    groebner_matrix[(34, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 151)] -= reduction_term;
    let factor = groebner_matrix[(34, 70)];
    groebner_matrix[(34, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(34, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(34, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(34, 194)] -= reduction_term;
    let factor = -groebner_matrix[(34, 73)];
    groebner_matrix[(34, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 176)] -= reduction_term;
    let factor = -groebner_matrix[(34, 76)];
    groebner_matrix[(34, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 142)] -= reduction_term;
    let factor = -groebner_matrix[(34, 77)];
    groebner_matrix[(34, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 143)] -= reduction_term;
    let factor = groebner_matrix[(34, 78)];
    groebner_matrix[(34, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 160)] -= reduction_term;
    let factor = -groebner_matrix[(34, 79)];
    groebner_matrix[(34, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 148)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_47(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(34, 80)];

    groebner_matrix[(34, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 152)] -= reduction_term;
    let factor = -groebner_matrix[(34, 81)];
    groebner_matrix[(34, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 157)] -= reduction_term;
    let factor = groebner_matrix[(34, 82)];
    groebner_matrix[(34, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(34, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(34, 127)] -= reduction_term;
    let factor = groebner_matrix[(34, 83)];
    groebner_matrix[(34, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(34, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(34, 175)] -= reduction_term;
    let factor = groebner_matrix[(34, 84)];
    groebner_matrix[(34, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 166)] -= reduction_term;
    let factor = groebner_matrix[(34, 85)];
    groebner_matrix[(34, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(34, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(34, 130)] -= reduction_term;
    let factor = -groebner_matrix[(34, 86)];
    groebner_matrix[(34, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(34, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(34, 172)] -= reduction_term;
    let factor = -groebner_matrix[(34, 87)];
    groebner_matrix[(34, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 163)] -= reduction_term;
    let factor = groebner_matrix[(34, 91)];
    groebner_matrix[(34, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 173)] -= reduction_term;
    let factor = -groebner_matrix[(34, 94)];
    groebner_matrix[(34, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 170)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_48(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(34, 97)];

    groebner_matrix[(34, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(34, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(34, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(34, 195)] -= reduction_term;
    let factor = groebner_matrix[(34, 100)];
    groebner_matrix[(34, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(34, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(34, 139)] -= reduction_term;
    let factor = groebner_matrix[(34, 103)];
    groebner_matrix[(34, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(34, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(34, 114)] -= reduction_term;
    let factor = -groebner_matrix[(34, 104)];
    groebner_matrix[(34, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(34, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(34, 186)] -= reduction_term;
    let factor = groebner_matrix[(34, 105)];
    groebner_matrix[(34, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(34, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(34, 185)] -= reduction_term;
    let factor = groebner_matrix[(34, 106)];
    groebner_matrix[(34, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(34, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(34, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(34, 195)] -= reduction_term;
    let factor = -groebner_matrix[(34, 109)];
    groebner_matrix[(34, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 184)] -= reduction_term;
    let factor = groebner_matrix[(34, 118)];
    groebner_matrix[(34, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(34, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(34, 135)] -= reduction_term;
    let factor = groebner_matrix[(34, 119)];
    groebner_matrix[(34, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(34, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(34, 183)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_49(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(34, 120)];

    groebner_matrix[(34, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(34, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(34, 182)] -= reduction_term;
    let factor = groebner_matrix[(34, 121)];
    groebner_matrix[(34, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(34, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(34, 138)] -= reduction_term;
    let factor = -groebner_matrix[(34, 122)];
    groebner_matrix[(34, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(34, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(34, 180)] -= reduction_term;
    let factor = groebner_matrix[(34, 123)];
    groebner_matrix[(34, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(34, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(34, 179)] -= reduction_term;
    let factor = groebner_matrix[(34, 127)];
    groebner_matrix[(34, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 181)] -= reduction_term;
    let factor = -groebner_matrix[(34, 130)];
    groebner_matrix[(34, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 178)] -= reduction_term;
    let factor = groebner_matrix[(34, 142)];
    groebner_matrix[(34, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(34, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(34, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(34, 196)] -= reduction_term;
    let factor = groebner_matrix[(34, 143)];
    groebner_matrix[(34, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(34, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(34, 176)] -= reduction_term;
    let factor = groebner_matrix[(34, 144)];
    groebner_matrix[(34, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(34, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(34, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(34, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_50(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(34, 146)];

    groebner_matrix[(34, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(34, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(34, 185)] -= reduction_term;
    let factor = groebner_matrix[(34, 147)];
    groebner_matrix[(34, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(34, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(34, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(34, 196)] -= reduction_term;
    let factor = groebner_matrix[(34, 148)];
    groebner_matrix[(34, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(34, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(34, 159)] -= reduction_term;
    let factor = -groebner_matrix[(34, 149)];
    groebner_matrix[(34, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(34, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(34, 195)] -= reduction_term;
    let factor = groebner_matrix[(34, 150)];
    groebner_matrix[(34, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(34, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(34, 194)] -= reduction_term;
    let factor = groebner_matrix[(34, 151)];
    groebner_matrix[(34, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(34, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(34, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(34, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 34);
    groebner_row31_000000000_f(groebner_matrix, 34);
    let factor = -groebner_matrix[(34, 154)];
    groebner_matrix[(34, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(34, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(34, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 34);
    groebner_row33_000000000_f(groebner_matrix, 34);
    let factor = groebner_matrix[(34, 163)];
    groebner_matrix[(34, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(34, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(34, 180)] -= reduction_term;
    let factor = groebner_matrix[(34, 164)];
    groebner_matrix[(34, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(34, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(34, 192)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_51(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(34, 165)];

    groebner_matrix[(34, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(34, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(34, 191)] -= reduction_term;
    let factor = groebner_matrix[(34, 166)];
    groebner_matrix[(34, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(34, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(34, 183)] -= reduction_term;
    let factor = -groebner_matrix[(34, 167)];
    groebner_matrix[(34, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(34, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(34, 189)] -= reduction_term;
    let factor = groebner_matrix[(34, 168)];
    groebner_matrix[(34, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(34, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(34, 188)] -= reduction_term;
    let factor = groebner_matrix[(34, 169)];
    groebner_matrix[(34, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(34, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(34, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(34, 196)] -= reduction_term;
    let factor = groebner_matrix[(34, 172)];
    groebner_matrix[(34, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(34, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(34, 190)] -= reduction_term;
    let factor = -groebner_matrix[(34, 175)];
    groebner_matrix[(34, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(34, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(34, 187)] -= reduction_term;
    s_polynomial35(groebner_matrix);
    let factor = -groebner_matrix[(35, 1)];
    groebner_matrix[(35, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(35, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(35, 179)] -= reduction_term;
    let factor = groebner_matrix[(35, 2)];
    groebner_matrix[(35, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(35, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(35, 24)] -= reduction_term;
    let factor = -groebner_matrix[(35, 3)];
    groebner_matrix[(35, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(35, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(35, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_52(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(35, 4)];
    groebner_matrix[(35, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(35, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(35, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(35, 190)] -= reduction_term;
    let factor = groebner_matrix[(35, 5)];
    groebner_matrix[(35, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(35, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(35, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(35, 188)] -= reduction_term;
    let factor = groebner_matrix[(35, 6)];
    groebner_matrix[(35, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(35, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(35, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(35, 189)] -= reduction_term;
    let factor = -groebner_matrix[(35, 10)];
    groebner_matrix[(35, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 163)] -= reduction_term;
    let factor = -groebner_matrix[(35, 11)];
    groebner_matrix[(35, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 164)] -= reduction_term;
    let factor = -groebner_matrix[(35, 12)];
    groebner_matrix[(35, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 165)] -= reduction_term;
    let factor = groebner_matrix[(35, 13)];
    groebner_matrix[(35, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(35, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(35, 30)] -= reduction_term;
    let factor = -groebner_matrix[(35, 14)];
    groebner_matrix[(35, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(35, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(35, 182)] -= reduction_term;
    let factor = -groebner_matrix[(35, 15)];
    groebner_matrix[(35, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 166)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_53(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(35, 18)];

    groebner_matrix[(35, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 167)] -= reduction_term;
    let factor = groebner_matrix[(35, 19)];
    groebner_matrix[(35, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(35, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(35, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(35, 192)] -= reduction_term;
    let factor = groebner_matrix[(35, 22)];
    groebner_matrix[(35, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(35, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(35, 123)] -= reduction_term;
    let factor = groebner_matrix[(35, 25)];
    groebner_matrix[(35, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(35, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(35, 35)] -= reduction_term;
    let factor = -groebner_matrix[(35, 26)];
    groebner_matrix[(35, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(35, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(35, 183)] -= reduction_term;
    let factor = groebner_matrix[(35, 27)];
    groebner_matrix[(35, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(35, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(35, 175)] -= reduction_term;
    let factor = -groebner_matrix[(35, 30)];
    groebner_matrix[(35, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 168)] -= reduction_term;
    let factor = groebner_matrix[(35, 39)];
    groebner_matrix[(35, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(35, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(35, 101)] -= reduction_term;
    let factor = groebner_matrix[(35, 40)];
    groebner_matrix[(35, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(35, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(35, 158)] -= reduction_term;
    let factor = groebner_matrix[(35, 41)];
    groebner_matrix[(35, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(35, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(35, 102)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_54(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(35, 42)];

    groebner_matrix[(35, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(35, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(35, 159)] -= reduction_term;
    let factor = groebner_matrix[(35, 43)];
    groebner_matrix[(35, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(35, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(35, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(35, 193)] -= reduction_term;
    let factor = groebner_matrix[(35, 44)];
    groebner_matrix[(35, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(35, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(35, 113)] -= reduction_term;
    let factor = groebner_matrix[(35, 45)];
    groebner_matrix[(35, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(35, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(35, 114)] -= reduction_term;
    let factor = groebner_matrix[(35, 46)];
    groebner_matrix[(35, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(35, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(35, 109)] -= reduction_term;
    let factor = groebner_matrix[(35, 47)];
    groebner_matrix[(35, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(35, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(35, 161)] -= reduction_term;
    let factor = -groebner_matrix[(35, 48)];
    groebner_matrix[(35, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 169)] -= reduction_term;
    let factor = groebner_matrix[(35, 49)];
    groebner_matrix[(35, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(35, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(35, 116)] -= reduction_term;
    let factor = -groebner_matrix[(35, 50)];
    groebner_matrix[(35, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(35, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(35, 154)] -= reduction_term;
    let factor = groebner_matrix[(35, 51)];
    groebner_matrix[(35, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(35, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(35, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_55(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(35, 52)];

    groebner_matrix[(35, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(35, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(35, 162)] -= reduction_term;
    let factor = groebner_matrix[(35, 53)];
    groebner_matrix[(35, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(35, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(35, 146)] -= reduction_term;
    let factor = groebner_matrix[(35, 54)];
    groebner_matrix[(35, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(35, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(35, 117)] -= reduction_term;
    let factor = -groebner_matrix[(35, 55)];
    groebner_matrix[(35, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(35, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(35, 159)] -= reduction_term;
    let factor = groebner_matrix[(35, 56)];
    groebner_matrix[(35, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(35, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(35, 158)] -= reduction_term;
    let factor = groebner_matrix[(35, 57)];
    groebner_matrix[(35, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(35, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(35, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(35, 188)] -= reduction_term;
    let factor = groebner_matrix[(35, 58)];
    groebner_matrix[(35, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(35, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(35, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(35, 189)] -= reduction_term;
    let factor = groebner_matrix[(35, 59)];
    groebner_matrix[(35, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(35, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(35, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(35, 191)] -= reduction_term;
    let factor = groebner_matrix[(35, 60)];
    groebner_matrix[(35, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(35, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(35, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(35, 192)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_56(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(35, 61)];

    groebner_matrix[(35, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(35, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(35, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(35, 194)] -= reduction_term;
    let factor = groebner_matrix[(35, 64)];
    groebner_matrix[(35, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 148)] -= reduction_term;
    let factor = groebner_matrix[(35, 65)];
    groebner_matrix[(35, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 149)] -= reduction_term;
    let factor = groebner_matrix[(35, 66)];
    groebner_matrix[(35, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 150)] -= reduction_term;
    let factor = groebner_matrix[(35, 67)];
    groebner_matrix[(35, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(35, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(35, 78)] -= reduction_term;
    let factor = -groebner_matrix[(35, 68)];
    groebner_matrix[(35, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(35, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(35, 185)] -= reduction_term;
    let factor = groebner_matrix[(35, 69)];
    groebner_matrix[(35, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 151)] -= reduction_term;
    let factor = groebner_matrix[(35, 70)];
    groebner_matrix[(35, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(35, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(35, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(35, 194)] -= reduction_term;
    let factor = -groebner_matrix[(35, 73)];
    groebner_matrix[(35, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 176)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_57(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(35, 76)];

    groebner_matrix[(35, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 142)] -= reduction_term;
    let factor = -groebner_matrix[(35, 77)];
    groebner_matrix[(35, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 143)] -= reduction_term;
    let factor = groebner_matrix[(35, 78)];
    groebner_matrix[(35, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 160)] -= reduction_term;
    let factor = -groebner_matrix[(35, 79)];
    groebner_matrix[(35, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 148)] -= reduction_term;
    let factor = -groebner_matrix[(35, 80)];
    groebner_matrix[(35, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 152)] -= reduction_term;
    let factor = -groebner_matrix[(35, 81)];
    groebner_matrix[(35, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 157)] -= reduction_term;
    let factor = groebner_matrix[(35, 82)];
    groebner_matrix[(35, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(35, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(35, 127)] -= reduction_term;
    let factor = groebner_matrix[(35, 83)];
    groebner_matrix[(35, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(35, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(35, 175)] -= reduction_term;
    let factor = groebner_matrix[(35, 84)];
    groebner_matrix[(35, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 166)] -= reduction_term;
    let factor = groebner_matrix[(35, 85)];
    groebner_matrix[(35, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(35, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(35, 130)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_58(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(35, 86)];

    groebner_matrix[(35, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(35, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(35, 172)] -= reduction_term;
    let factor = -groebner_matrix[(35, 87)];
    groebner_matrix[(35, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 163)] -= reduction_term;
    let factor = groebner_matrix[(35, 91)];
    groebner_matrix[(35, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 173)] -= reduction_term;
    let factor = -groebner_matrix[(35, 94)];
    groebner_matrix[(35, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 170)] -= reduction_term;
    let factor = groebner_matrix[(35, 97)];
    groebner_matrix[(35, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(35, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(35, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(35, 195)] -= reduction_term;
    let factor = groebner_matrix[(35, 100)];
    groebner_matrix[(35, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(35, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(35, 139)] -= reduction_term;
    let factor = groebner_matrix[(35, 103)];
    groebner_matrix[(35, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(35, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(35, 114)] -= reduction_term;
    let factor = -groebner_matrix[(35, 104)];
    groebner_matrix[(35, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(35, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(35, 186)] -= reduction_term;
    let factor = groebner_matrix[(35, 105)];
    groebner_matrix[(35, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(35, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(35, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_59(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(35, 106)];

    groebner_matrix[(35, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(35, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(35, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(35, 195)] -= reduction_term;
    let factor = -groebner_matrix[(35, 109)];
    groebner_matrix[(35, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 184)] -= reduction_term;
    let factor = groebner_matrix[(35, 118)];
    groebner_matrix[(35, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(35, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(35, 135)] -= reduction_term;
    let factor = groebner_matrix[(35, 119)];
    groebner_matrix[(35, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(35, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(35, 183)] -= reduction_term;
    let factor = -groebner_matrix[(35, 120)];
    groebner_matrix[(35, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(35, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(35, 182)] -= reduction_term;
    let factor = groebner_matrix[(35, 121)];
    groebner_matrix[(35, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(35, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(35, 138)] -= reduction_term;
    let factor = -groebner_matrix[(35, 122)];
    groebner_matrix[(35, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(35, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(35, 180)] -= reduction_term;
    let factor = groebner_matrix[(35, 123)];
    groebner_matrix[(35, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(35, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(35, 179)] -= reduction_term;
    let factor = groebner_matrix[(35, 127)];
    groebner_matrix[(35, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 181)] -= reduction_term;
    let factor = -groebner_matrix[(35, 130)];
    groebner_matrix[(35, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 178)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_60(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(35, 142)];

    groebner_matrix[(35, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(35, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(35, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(35, 196)] -= reduction_term;
    let factor = groebner_matrix[(35, 143)];
    groebner_matrix[(35, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(35, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(35, 176)] -= reduction_term;
    let factor = groebner_matrix[(35, 144)];
    groebner_matrix[(35, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(35, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(35, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(35, 196)] -= reduction_term;
    let factor = groebner_matrix[(35, 146)];
    groebner_matrix[(35, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(35, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(35, 185)] -= reduction_term;
    let factor = groebner_matrix[(35, 147)];
    groebner_matrix[(35, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(35, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(35, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(35, 196)] -= reduction_term;
    let factor = groebner_matrix[(35, 148)];
    groebner_matrix[(35, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(35, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(35, 159)] -= reduction_term;
    let factor = -groebner_matrix[(35, 149)];
    groebner_matrix[(35, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(35, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(35, 195)] -= reduction_term;
    let factor = groebner_matrix[(35, 150)];
    groebner_matrix[(35, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(35, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(35, 194)] -= reduction_term;
    let factor = groebner_matrix[(35, 151)];
    groebner_matrix[(35, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(35, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(35, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(35, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 35);
}

#[inline]
fn compute_basis_stage_61(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row31_000000000_f(groebner_matrix, 35);
    let factor = -groebner_matrix[(35, 154)];
    groebner_matrix[(35, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(35, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(35, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 35);
    groebner_row33_000000000_f(groebner_matrix, 35);
    groebner_row34_000000000_f(groebner_matrix, 35);
    let factor = groebner_matrix[(35, 163)];
    groebner_matrix[(35, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(35, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(35, 180)] -= reduction_term;
    let factor = groebner_matrix[(35, 164)];
    groebner_matrix[(35, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(35, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(35, 192)] -= reduction_term;
    let factor = -groebner_matrix[(35, 165)];
    groebner_matrix[(35, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(35, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(35, 191)] -= reduction_term;
    let factor = groebner_matrix[(35, 166)];
    groebner_matrix[(35, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(35, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(35, 183)] -= reduction_term;
    let factor = -groebner_matrix[(35, 167)];
    groebner_matrix[(35, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(35, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(35, 189)] -= reduction_term;
    let factor = groebner_matrix[(35, 168)];
    groebner_matrix[(35, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(35, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(35, 188)] -= reduction_term;
    let factor = groebner_matrix[(35, 169)];
    groebner_matrix[(35, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(35, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(35, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(35, 196)] -= reduction_term;
    let factor = groebner_matrix[(35, 172)];
    groebner_matrix[(35, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(35, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(35, 190)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_62(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(35, 175)];

    groebner_matrix[(35, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(35, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(35, 187)] -= reduction_term;
    s_polynomial36(groebner_matrix);
    let factor = -groebner_matrix[(36, 1)];
    groebner_matrix[(36, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(36, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(36, 179)] -= reduction_term;
    let factor = groebner_matrix[(36, 2)];
    groebner_matrix[(36, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(36, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(36, 24)] -= reduction_term;
    let factor = -groebner_matrix[(36, 3)];
    groebner_matrix[(36, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(36, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(36, 180)] -= reduction_term;
    let factor = groebner_matrix[(36, 4)];
    groebner_matrix[(36, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(36, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(36, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(36, 190)] -= reduction_term;
    let factor = groebner_matrix[(36, 5)];
    groebner_matrix[(36, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(36, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(36, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(36, 188)] -= reduction_term;
    let factor = groebner_matrix[(36, 6)];
    groebner_matrix[(36, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(36, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(36, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(36, 189)] -= reduction_term;
    let factor = -groebner_matrix[(36, 10)];
    groebner_matrix[(36, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 163)] -= reduction_term;
    let factor = -groebner_matrix[(36, 11)];
    groebner_matrix[(36, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 164)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_63(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(36, 12)];

    groebner_matrix[(36, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 165)] -= reduction_term;
    let factor = groebner_matrix[(36, 13)];
    groebner_matrix[(36, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(36, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(36, 30)] -= reduction_term;
    let factor = -groebner_matrix[(36, 14)];
    groebner_matrix[(36, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(36, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(36, 182)] -= reduction_term;
    let factor = -groebner_matrix[(36, 15)];
    groebner_matrix[(36, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 166)] -= reduction_term;
    let factor = -groebner_matrix[(36, 18)];
    groebner_matrix[(36, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 167)] -= reduction_term;
    let factor = groebner_matrix[(36, 19)];
    groebner_matrix[(36, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(36, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(36, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(36, 192)] -= reduction_term;
    let factor = groebner_matrix[(36, 22)];
    groebner_matrix[(36, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(36, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(36, 123)] -= reduction_term;
    let factor = groebner_matrix[(36, 25)];
    groebner_matrix[(36, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(36, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(36, 35)] -= reduction_term;
    let factor = -groebner_matrix[(36, 26)];
    groebner_matrix[(36, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(36, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(36, 183)] -= reduction_term;
    let factor = groebner_matrix[(36, 27)];
    groebner_matrix[(36, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(36, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(36, 175)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_64(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(36, 30)];

    groebner_matrix[(36, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 168)] -= reduction_term;
    let factor = groebner_matrix[(36, 39)];
    groebner_matrix[(36, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(36, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(36, 101)] -= reduction_term;
    let factor = groebner_matrix[(36, 40)];
    groebner_matrix[(36, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(36, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(36, 158)] -= reduction_term;
    let factor = groebner_matrix[(36, 41)];
    groebner_matrix[(36, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(36, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(36, 102)] -= reduction_term;
    let factor = groebner_matrix[(36, 42)];
    groebner_matrix[(36, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(36, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(36, 159)] -= reduction_term;
    let factor = groebner_matrix[(36, 43)];
    groebner_matrix[(36, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(36, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(36, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(36, 193)] -= reduction_term;
    let factor = groebner_matrix[(36, 44)];
    groebner_matrix[(36, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(36, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(36, 113)] -= reduction_term;
    let factor = groebner_matrix[(36, 45)];
    groebner_matrix[(36, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(36, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(36, 114)] -= reduction_term;
    let factor = groebner_matrix[(36, 46)];
    groebner_matrix[(36, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(36, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(36, 109)] -= reduction_term;
    let factor = groebner_matrix[(36, 47)];
    groebner_matrix[(36, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(36, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(36, 161)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_65(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(36, 48)];

    groebner_matrix[(36, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 169)] -= reduction_term;
    let factor = groebner_matrix[(36, 49)];
    groebner_matrix[(36, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(36, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(36, 116)] -= reduction_term;
    let factor = -groebner_matrix[(36, 50)];
    groebner_matrix[(36, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(36, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(36, 154)] -= reduction_term;
    let factor = groebner_matrix[(36, 51)];
    groebner_matrix[(36, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(36, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(36, 114)] -= reduction_term;
    let factor = groebner_matrix[(36, 52)];
    groebner_matrix[(36, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(36, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(36, 162)] -= reduction_term;
    let factor = groebner_matrix[(36, 53)];
    groebner_matrix[(36, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(36, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(36, 146)] -= reduction_term;
    let factor = groebner_matrix[(36, 54)];
    groebner_matrix[(36, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(36, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(36, 117)] -= reduction_term;
    let factor = -groebner_matrix[(36, 55)];
    groebner_matrix[(36, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(36, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(36, 159)] -= reduction_term;
    let factor = groebner_matrix[(36, 56)];
    groebner_matrix[(36, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(36, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(36, 158)] -= reduction_term;
    let factor = groebner_matrix[(36, 57)];
    groebner_matrix[(36, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(36, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(36, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(36, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_66(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(36, 58)];

    groebner_matrix[(36, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(36, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(36, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(36, 189)] -= reduction_term;
    let factor = groebner_matrix[(36, 59)];
    groebner_matrix[(36, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(36, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(36, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(36, 191)] -= reduction_term;
    let factor = groebner_matrix[(36, 60)];
    groebner_matrix[(36, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(36, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(36, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(36, 192)] -= reduction_term;
    let factor = groebner_matrix[(36, 61)];
    groebner_matrix[(36, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(36, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(36, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(36, 194)] -= reduction_term;
    let factor = groebner_matrix[(36, 64)];
    groebner_matrix[(36, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 148)] -= reduction_term;
    let factor = groebner_matrix[(36, 65)];
    groebner_matrix[(36, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 149)] -= reduction_term;
    let factor = groebner_matrix[(36, 66)];
    groebner_matrix[(36, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 150)] -= reduction_term;
    let factor = groebner_matrix[(36, 67)];
    groebner_matrix[(36, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(36, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(36, 78)] -= reduction_term;
    let factor = -groebner_matrix[(36, 68)];
    groebner_matrix[(36, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(36, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(36, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_67(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(36, 69)];

    groebner_matrix[(36, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 151)] -= reduction_term;
    let factor = groebner_matrix[(36, 70)];
    groebner_matrix[(36, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(36, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(36, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(36, 194)] -= reduction_term;
    let factor = -groebner_matrix[(36, 73)];
    groebner_matrix[(36, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 176)] -= reduction_term;
    let factor = -groebner_matrix[(36, 76)];
    groebner_matrix[(36, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 142)] -= reduction_term;
    let factor = -groebner_matrix[(36, 77)];
    groebner_matrix[(36, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 143)] -= reduction_term;
    let factor = groebner_matrix[(36, 78)];
    groebner_matrix[(36, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 160)] -= reduction_term;
    let factor = -groebner_matrix[(36, 79)];
    groebner_matrix[(36, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 148)] -= reduction_term;
    let factor = -groebner_matrix[(36, 80)];
    groebner_matrix[(36, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 152)] -= reduction_term;
    let factor = -groebner_matrix[(36, 81)];
    groebner_matrix[(36, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 157)] -= reduction_term;
    let factor = groebner_matrix[(36, 82)];
    groebner_matrix[(36, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(36, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(36, 127)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_68(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(36, 83)];

    groebner_matrix[(36, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(36, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(36, 175)] -= reduction_term;
    let factor = groebner_matrix[(36, 84)];
    groebner_matrix[(36, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 166)] -= reduction_term;
    let factor = groebner_matrix[(36, 85)];
    groebner_matrix[(36, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(36, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(36, 130)] -= reduction_term;
    let factor = -groebner_matrix[(36, 86)];
    groebner_matrix[(36, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(36, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(36, 172)] -= reduction_term;
    let factor = -groebner_matrix[(36, 87)];
    groebner_matrix[(36, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 163)] -= reduction_term;
    let factor = groebner_matrix[(36, 91)];
    groebner_matrix[(36, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 173)] -= reduction_term;
    let factor = -groebner_matrix[(36, 94)];
    groebner_matrix[(36, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 170)] -= reduction_term;
    let factor = groebner_matrix[(36, 97)];
    groebner_matrix[(36, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(36, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(36, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(36, 195)] -= reduction_term;
    let factor = groebner_matrix[(36, 100)];
    groebner_matrix[(36, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(36, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(36, 139)] -= reduction_term;
    let factor = groebner_matrix[(36, 103)];
    groebner_matrix[(36, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(36, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(36, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_69(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(36, 104)];

    groebner_matrix[(36, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(36, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(36, 186)] -= reduction_term;
    let factor = groebner_matrix[(36, 105)];
    groebner_matrix[(36, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(36, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(36, 185)] -= reduction_term;
    let factor = groebner_matrix[(36, 106)];
    groebner_matrix[(36, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(36, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(36, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(36, 195)] -= reduction_term;
    let factor = -groebner_matrix[(36, 109)];
    groebner_matrix[(36, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 184)] -= reduction_term;
    let factor = groebner_matrix[(36, 118)];
    groebner_matrix[(36, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(36, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(36, 135)] -= reduction_term;
    let factor = groebner_matrix[(36, 119)];
    groebner_matrix[(36, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(36, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(36, 183)] -= reduction_term;
    let factor = -groebner_matrix[(36, 120)];
    groebner_matrix[(36, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(36, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(36, 182)] -= reduction_term;
    let factor = groebner_matrix[(36, 121)];
    groebner_matrix[(36, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(36, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(36, 138)] -= reduction_term;
    let factor = -groebner_matrix[(36, 122)];
    groebner_matrix[(36, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(36, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(36, 180)] -= reduction_term;
    let factor = groebner_matrix[(36, 123)];
    groebner_matrix[(36, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(36, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(36, 179)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_70(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(36, 127)];

    groebner_matrix[(36, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 181)] -= reduction_term;
    let factor = -groebner_matrix[(36, 130)];
    groebner_matrix[(36, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 178)] -= reduction_term;
    let factor = groebner_matrix[(36, 142)];
    groebner_matrix[(36, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(36, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(36, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(36, 196)] -= reduction_term;
    let factor = groebner_matrix[(36, 143)];
    groebner_matrix[(36, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(36, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(36, 176)] -= reduction_term;
    let factor = groebner_matrix[(36, 144)];
    groebner_matrix[(36, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(36, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(36, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(36, 196)] -= reduction_term;
    let factor = groebner_matrix[(36, 146)];
    groebner_matrix[(36, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(36, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(36, 185)] -= reduction_term;
    let factor = groebner_matrix[(36, 147)];
    groebner_matrix[(36, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(36, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(36, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(36, 196)] -= reduction_term;
    let factor = groebner_matrix[(36, 148)];
    groebner_matrix[(36, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(36, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(36, 159)] -= reduction_term;
    let factor = -groebner_matrix[(36, 149)];
    groebner_matrix[(36, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(36, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(36, 195)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_71(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(36, 150)];

    groebner_matrix[(36, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(36, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(36, 194)] -= reduction_term;
    let factor = groebner_matrix[(36, 151)];
    groebner_matrix[(36, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(36, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(36, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(36, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 36);
    groebner_row31_000000000_f(groebner_matrix, 36);
    let factor = -groebner_matrix[(36, 154)];
    groebner_matrix[(36, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(36, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(36, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 36);
    groebner_row33_000000000_f(groebner_matrix, 36);
    groebner_row34_000000000_f(groebner_matrix, 36);
    groebner_row35_000000000_f(groebner_matrix, 36);
    let factor = groebner_matrix[(36, 163)];
    groebner_matrix[(36, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(36, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(36, 180)] -= reduction_term;
    let factor = groebner_matrix[(36, 164)];
    groebner_matrix[(36, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(36, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(36, 192)] -= reduction_term;
    let factor = -groebner_matrix[(36, 165)];
    groebner_matrix[(36, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(36, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(36, 191)] -= reduction_term;
    let factor = groebner_matrix[(36, 166)];
    groebner_matrix[(36, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(36, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(36, 183)] -= reduction_term;
    let factor = -groebner_matrix[(36, 167)];
    groebner_matrix[(36, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(36, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(36, 189)] -= reduction_term;
    let factor = groebner_matrix[(36, 168)];
    groebner_matrix[(36, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(36, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(36, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_72(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(36, 169)];

    groebner_matrix[(36, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(36, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(36, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(36, 196)] -= reduction_term;
    let factor = groebner_matrix[(36, 172)];
    groebner_matrix[(36, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(36, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(36, 190)] -= reduction_term;
    let factor = -groebner_matrix[(36, 175)];
    groebner_matrix[(36, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(36, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(36, 187)] -= reduction_term;
    s_polynomial37(groebner_matrix);
    let factor = -groebner_matrix[(37, 1)];
    groebner_matrix[(37, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(37, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(37, 179)] -= reduction_term;
    let factor = groebner_matrix[(37, 2)];
    groebner_matrix[(37, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(37, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(37, 24)] -= reduction_term;
    let factor = -groebner_matrix[(37, 3)];
    groebner_matrix[(37, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(37, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(37, 180)] -= reduction_term;
    let factor = groebner_matrix[(37, 4)];
    groebner_matrix[(37, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(37, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(37, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(37, 190)] -= reduction_term;
    let factor = groebner_matrix[(37, 5)];
    groebner_matrix[(37, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(37, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(37, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(37, 188)] -= reduction_term;
    let factor = groebner_matrix[(37, 6)];
    groebner_matrix[(37, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(37, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(37, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(37, 189)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_73(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(37, 10)];
    groebner_matrix[(37, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 163)] -= reduction_term;
    let factor = -groebner_matrix[(37, 11)];
    groebner_matrix[(37, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 164)] -= reduction_term;
    let factor = -groebner_matrix[(37, 12)];
    groebner_matrix[(37, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 165)] -= reduction_term;
    let factor = groebner_matrix[(37, 13)];
    groebner_matrix[(37, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(37, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(37, 30)] -= reduction_term;
    let factor = -groebner_matrix[(37, 14)];
    groebner_matrix[(37, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(37, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(37, 182)] -= reduction_term;
    let factor = -groebner_matrix[(37, 15)];
    groebner_matrix[(37, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 166)] -= reduction_term;
    let factor = -groebner_matrix[(37, 18)];
    groebner_matrix[(37, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 167)] -= reduction_term;
    let factor = groebner_matrix[(37, 19)];
    groebner_matrix[(37, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(37, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(37, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(37, 192)] -= reduction_term;
    let factor = groebner_matrix[(37, 22)];
    groebner_matrix[(37, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(37, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(37, 123)] -= reduction_term;
    let factor = groebner_matrix[(37, 25)];
    groebner_matrix[(37, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(37, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(37, 35)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_74(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(37, 26)];
    groebner_matrix[(37, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(37, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(37, 183)] -= reduction_term;
    let factor = groebner_matrix[(37, 27)];
    groebner_matrix[(37, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(37, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(37, 175)] -= reduction_term;
    let factor = -groebner_matrix[(37, 30)];
    groebner_matrix[(37, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 168)] -= reduction_term;
    let factor = groebner_matrix[(37, 39)];
    groebner_matrix[(37, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(37, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(37, 101)] -= reduction_term;
    let factor = groebner_matrix[(37, 40)];
    groebner_matrix[(37, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(37, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(37, 158)] -= reduction_term;
    let factor = groebner_matrix[(37, 41)];
    groebner_matrix[(37, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(37, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(37, 102)] -= reduction_term;
    let factor = groebner_matrix[(37, 42)];
    groebner_matrix[(37, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(37, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(37, 159)] -= reduction_term;
    let factor = groebner_matrix[(37, 43)];
    groebner_matrix[(37, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(37, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(37, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(37, 193)] -= reduction_term;
    let factor = groebner_matrix[(37, 44)];
    groebner_matrix[(37, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(37, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(37, 113)] -= reduction_term;
    let factor = groebner_matrix[(37, 45)];
    groebner_matrix[(37, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(37, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(37, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_75(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(37, 46)];
    groebner_matrix[(37, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(37, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(37, 109)] -= reduction_term;
    let factor = groebner_matrix[(37, 47)];
    groebner_matrix[(37, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(37, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(37, 161)] -= reduction_term;
    let factor = -groebner_matrix[(37, 48)];
    groebner_matrix[(37, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 169)] -= reduction_term;
    let factor = groebner_matrix[(37, 49)];
    groebner_matrix[(37, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(37, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(37, 116)] -= reduction_term;
    let factor = -groebner_matrix[(37, 50)];
    groebner_matrix[(37, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(37, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(37, 154)] -= reduction_term;
    let factor = groebner_matrix[(37, 51)];
    groebner_matrix[(37, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(37, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(37, 114)] -= reduction_term;
    let factor = groebner_matrix[(37, 52)];
    groebner_matrix[(37, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(37, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(37, 162)] -= reduction_term;
    let factor = groebner_matrix[(37, 53)];
    groebner_matrix[(37, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(37, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(37, 146)] -= reduction_term;
    let factor = groebner_matrix[(37, 54)];
    groebner_matrix[(37, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(37, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(37, 117)] -= reduction_term;
    let factor = -groebner_matrix[(37, 55)];
    groebner_matrix[(37, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(37, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(37, 159)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_76(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(37, 56)];

    groebner_matrix[(37, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(37, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(37, 158)] -= reduction_term;
    let factor = groebner_matrix[(37, 57)];
    groebner_matrix[(37, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(37, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(37, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(37, 188)] -= reduction_term;
    let factor = groebner_matrix[(37, 58)];
    groebner_matrix[(37, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(37, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(37, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(37, 189)] -= reduction_term;
    let factor = groebner_matrix[(37, 59)];
    groebner_matrix[(37, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(37, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(37, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(37, 191)] -= reduction_term;
    let factor = groebner_matrix[(37, 60)];
    groebner_matrix[(37, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(37, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(37, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(37, 192)] -= reduction_term;
    let factor = groebner_matrix[(37, 61)];
    groebner_matrix[(37, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(37, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(37, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(37, 194)] -= reduction_term;
    let factor = groebner_matrix[(37, 64)];
    groebner_matrix[(37, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 148)] -= reduction_term;
    let factor = groebner_matrix[(37, 65)];
    groebner_matrix[(37, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 149)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_77(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(37, 66)];

    groebner_matrix[(37, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 150)] -= reduction_term;
    let factor = groebner_matrix[(37, 67)];
    groebner_matrix[(37, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(37, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(37, 78)] -= reduction_term;
    let factor = -groebner_matrix[(37, 68)];
    groebner_matrix[(37, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(37, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(37, 185)] -= reduction_term;
    let factor = groebner_matrix[(37, 69)];
    groebner_matrix[(37, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 151)] -= reduction_term;
    let factor = groebner_matrix[(37, 70)];
    groebner_matrix[(37, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(37, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(37, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(37, 194)] -= reduction_term;
    let factor = -groebner_matrix[(37, 73)];
    groebner_matrix[(37, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 176)] -= reduction_term;
    let factor = -groebner_matrix[(37, 76)];
    groebner_matrix[(37, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 142)] -= reduction_term;
    let factor = -groebner_matrix[(37, 77)];
    groebner_matrix[(37, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 143)] -= reduction_term;
    let factor = groebner_matrix[(37, 78)];
    groebner_matrix[(37, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 160)] -= reduction_term;
    let factor = -groebner_matrix[(37, 79)];
    groebner_matrix[(37, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 148)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_78(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(37, 80)];

    groebner_matrix[(37, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 152)] -= reduction_term;
    let factor = -groebner_matrix[(37, 81)];
    groebner_matrix[(37, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 157)] -= reduction_term;
    let factor = groebner_matrix[(37, 82)];
    groebner_matrix[(37, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(37, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(37, 127)] -= reduction_term;
    let factor = groebner_matrix[(37, 83)];
    groebner_matrix[(37, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(37, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(37, 175)] -= reduction_term;
    let factor = groebner_matrix[(37, 84)];
    groebner_matrix[(37, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 166)] -= reduction_term;
    let factor = groebner_matrix[(37, 85)];
    groebner_matrix[(37, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(37, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(37, 130)] -= reduction_term;
    let factor = -groebner_matrix[(37, 86)];
    groebner_matrix[(37, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(37, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(37, 172)] -= reduction_term;
    let factor = -groebner_matrix[(37, 87)];
    groebner_matrix[(37, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 163)] -= reduction_term;
    let factor = groebner_matrix[(37, 91)];
    groebner_matrix[(37, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 173)] -= reduction_term;
    let factor = -groebner_matrix[(37, 94)];
    groebner_matrix[(37, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 170)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_79(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(37, 97)];

    groebner_matrix[(37, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(37, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(37, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(37, 195)] -= reduction_term;
    let factor = groebner_matrix[(37, 100)];
    groebner_matrix[(37, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(37, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(37, 139)] -= reduction_term;
    let factor = groebner_matrix[(37, 103)];
    groebner_matrix[(37, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(37, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(37, 114)] -= reduction_term;
    let factor = -groebner_matrix[(37, 104)];
    groebner_matrix[(37, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(37, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(37, 186)] -= reduction_term;
    let factor = groebner_matrix[(37, 105)];
    groebner_matrix[(37, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(37, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(37, 185)] -= reduction_term;
    let factor = groebner_matrix[(37, 106)];
    groebner_matrix[(37, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(37, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(37, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(37, 195)] -= reduction_term;
    let factor = -groebner_matrix[(37, 109)];
    groebner_matrix[(37, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 184)] -= reduction_term;
    let factor = groebner_matrix[(37, 118)];
    groebner_matrix[(37, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(37, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(37, 135)] -= reduction_term;
    let factor = groebner_matrix[(37, 119)];
    groebner_matrix[(37, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(37, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(37, 183)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_80(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(37, 120)];

    groebner_matrix[(37, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(37, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(37, 182)] -= reduction_term;
    let factor = groebner_matrix[(37, 121)];
    groebner_matrix[(37, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(37, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(37, 138)] -= reduction_term;
    let factor = -groebner_matrix[(37, 122)];
    groebner_matrix[(37, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(37, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(37, 180)] -= reduction_term;
    let factor = groebner_matrix[(37, 123)];
    groebner_matrix[(37, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(37, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(37, 179)] -= reduction_term;
    let factor = groebner_matrix[(37, 127)];
    groebner_matrix[(37, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 181)] -= reduction_term;
    let factor = -groebner_matrix[(37, 130)];
    groebner_matrix[(37, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 178)] -= reduction_term;
    let factor = groebner_matrix[(37, 142)];
    groebner_matrix[(37, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(37, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(37, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(37, 196)] -= reduction_term;
    let factor = groebner_matrix[(37, 143)];
    groebner_matrix[(37, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(37, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(37, 176)] -= reduction_term;
    let factor = groebner_matrix[(37, 144)];
    groebner_matrix[(37, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(37, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(37, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(37, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_81(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(37, 146)];

    groebner_matrix[(37, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(37, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(37, 185)] -= reduction_term;
    let factor = groebner_matrix[(37, 147)];
    groebner_matrix[(37, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(37, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(37, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(37, 196)] -= reduction_term;
    let factor = groebner_matrix[(37, 148)];
    groebner_matrix[(37, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(37, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(37, 159)] -= reduction_term;
    let factor = -groebner_matrix[(37, 149)];
    groebner_matrix[(37, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(37, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(37, 195)] -= reduction_term;
    let factor = groebner_matrix[(37, 150)];
    groebner_matrix[(37, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(37, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(37, 194)] -= reduction_term;
    let factor = groebner_matrix[(37, 151)];
    groebner_matrix[(37, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(37, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(37, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(37, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 37);
    groebner_row31_000000000_f(groebner_matrix, 37);
    let factor = -groebner_matrix[(37, 154)];
    groebner_matrix[(37, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(37, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(37, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 37);
    groebner_row33_000000000_f(groebner_matrix, 37);
    groebner_row34_000000000_f(groebner_matrix, 37);
    groebner_row35_000000000_f(groebner_matrix, 37);
    groebner_row36_000000000_f(groebner_matrix, 37);
    let factor = groebner_matrix[(37, 163)];
    groebner_matrix[(37, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(37, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(37, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_82(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(37, 164)];

    groebner_matrix[(37, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(37, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(37, 192)] -= reduction_term;
    let factor = -groebner_matrix[(37, 165)];
    groebner_matrix[(37, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(37, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(37, 191)] -= reduction_term;
    let factor = groebner_matrix[(37, 166)];
    groebner_matrix[(37, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(37, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(37, 183)] -= reduction_term;
    let factor = -groebner_matrix[(37, 167)];
    groebner_matrix[(37, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(37, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(37, 189)] -= reduction_term;
    let factor = groebner_matrix[(37, 168)];
    groebner_matrix[(37, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(37, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(37, 188)] -= reduction_term;
    let factor = groebner_matrix[(37, 169)];
    groebner_matrix[(37, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(37, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(37, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(37, 196)] -= reduction_term;
    let factor = groebner_matrix[(37, 172)];
    groebner_matrix[(37, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(37, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(37, 190)] -= reduction_term;
    let factor = -groebner_matrix[(37, 175)];
    groebner_matrix[(37, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(37, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(37, 187)] -= reduction_term;
    s_polynomial38(groebner_matrix);
    let factor = -groebner_matrix[(38, 1)];
    groebner_matrix[(38, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(38, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(38, 179)] -= reduction_term;
    let factor = groebner_matrix[(38, 2)];
    groebner_matrix[(38, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(38, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(38, 24)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_83(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(38, 3)];
    groebner_matrix[(38, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(38, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(38, 180)] -= reduction_term;
    let factor = groebner_matrix[(38, 4)];
    groebner_matrix[(38, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(38, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(38, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(38, 190)] -= reduction_term;
    let factor = groebner_matrix[(38, 5)];
    groebner_matrix[(38, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(38, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(38, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(38, 188)] -= reduction_term;
    let factor = groebner_matrix[(38, 6)];
    groebner_matrix[(38, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(38, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(38, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(38, 189)] -= reduction_term;
    let factor = -groebner_matrix[(38, 10)];
    groebner_matrix[(38, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 163)] -= reduction_term;
    let factor = -groebner_matrix[(38, 11)];
    groebner_matrix[(38, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 164)] -= reduction_term;
    let factor = -groebner_matrix[(38, 12)];
    groebner_matrix[(38, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 165)] -= reduction_term;
    let factor = groebner_matrix[(38, 13)];
    groebner_matrix[(38, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(38, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(38, 30)] -= reduction_term;
    let factor = -groebner_matrix[(38, 14)];
    groebner_matrix[(38, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(38, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(38, 182)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_84(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(38, 15)];

    groebner_matrix[(38, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 166)] -= reduction_term;
    let factor = -groebner_matrix[(38, 18)];
    groebner_matrix[(38, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 167)] -= reduction_term;
    let factor = groebner_matrix[(38, 19)];
    groebner_matrix[(38, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(38, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(38, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(38, 192)] -= reduction_term;
    let factor = groebner_matrix[(38, 22)];
    groebner_matrix[(38, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(38, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(38, 123)] -= reduction_term;
    let factor = groebner_matrix[(38, 25)];
    groebner_matrix[(38, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(38, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(38, 35)] -= reduction_term;
    let factor = -groebner_matrix[(38, 26)];
    groebner_matrix[(38, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(38, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(38, 183)] -= reduction_term;
    let factor = groebner_matrix[(38, 27)];
    groebner_matrix[(38, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(38, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(38, 175)] -= reduction_term;
    let factor = -groebner_matrix[(38, 30)];
    groebner_matrix[(38, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 168)] -= reduction_term;
    let factor = groebner_matrix[(38, 39)];
    groebner_matrix[(38, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(38, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(38, 101)] -= reduction_term;
    let factor = groebner_matrix[(38, 40)];
    groebner_matrix[(38, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(38, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(38, 158)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_85(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(38, 41)];

    groebner_matrix[(38, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(38, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(38, 102)] -= reduction_term;
    let factor = groebner_matrix[(38, 42)];
    groebner_matrix[(38, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(38, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(38, 159)] -= reduction_term;
    let factor = groebner_matrix[(38, 43)];
    groebner_matrix[(38, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(38, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(38, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(38, 193)] -= reduction_term;
    let factor = groebner_matrix[(38, 44)];
    groebner_matrix[(38, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(38, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(38, 113)] -= reduction_term;
    let factor = groebner_matrix[(38, 45)];
    groebner_matrix[(38, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(38, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(38, 114)] -= reduction_term;
    let factor = groebner_matrix[(38, 46)];
    groebner_matrix[(38, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(38, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(38, 109)] -= reduction_term;
    let factor = groebner_matrix[(38, 47)];
    groebner_matrix[(38, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(38, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(38, 161)] -= reduction_term;
    let factor = -groebner_matrix[(38, 48)];
    groebner_matrix[(38, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 169)] -= reduction_term;
    let factor = groebner_matrix[(38, 49)];
    groebner_matrix[(38, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(38, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(38, 116)] -= reduction_term;
    let factor = -groebner_matrix[(38, 50)];
    groebner_matrix[(38, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(38, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(38, 154)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_86(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(38, 51)];

    groebner_matrix[(38, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(38, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(38, 114)] -= reduction_term;
    let factor = groebner_matrix[(38, 52)];
    groebner_matrix[(38, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(38, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(38, 162)] -= reduction_term;
    let factor = groebner_matrix[(38, 53)];
    groebner_matrix[(38, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(38, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(38, 146)] -= reduction_term;
    let factor = groebner_matrix[(38, 54)];
    groebner_matrix[(38, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(38, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(38, 117)] -= reduction_term;
    let factor = -groebner_matrix[(38, 55)];
    groebner_matrix[(38, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(38, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(38, 159)] -= reduction_term;
    let factor = groebner_matrix[(38, 56)];
    groebner_matrix[(38, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(38, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(38, 158)] -= reduction_term;
    let factor = groebner_matrix[(38, 57)];
    groebner_matrix[(38, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(38, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(38, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(38, 188)] -= reduction_term;
    let factor = groebner_matrix[(38, 58)];
    groebner_matrix[(38, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(38, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(38, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(38, 189)] -= reduction_term;
    let factor = groebner_matrix[(38, 59)];
    groebner_matrix[(38, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(38, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(38, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(38, 191)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_87(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(38, 60)];

    groebner_matrix[(38, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(38, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(38, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(38, 192)] -= reduction_term;
    let factor = groebner_matrix[(38, 61)];
    groebner_matrix[(38, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(38, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(38, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(38, 194)] -= reduction_term;
    let factor = groebner_matrix[(38, 64)];
    groebner_matrix[(38, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 148)] -= reduction_term;
    let factor = groebner_matrix[(38, 65)];
    groebner_matrix[(38, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 149)] -= reduction_term;
    let factor = groebner_matrix[(38, 66)];
    groebner_matrix[(38, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 150)] -= reduction_term;
    let factor = groebner_matrix[(38, 67)];
    groebner_matrix[(38, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(38, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(38, 78)] -= reduction_term;
    let factor = -groebner_matrix[(38, 68)];
    groebner_matrix[(38, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(38, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(38, 185)] -= reduction_term;
    let factor = groebner_matrix[(38, 69)];
    groebner_matrix[(38, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 151)] -= reduction_term;
    let factor = groebner_matrix[(38, 70)];
    groebner_matrix[(38, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(38, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(38, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(38, 194)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_88(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(38, 73)];

    groebner_matrix[(38, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 176)] -= reduction_term;
    let factor = -groebner_matrix[(38, 76)];
    groebner_matrix[(38, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 142)] -= reduction_term;
    let factor = -groebner_matrix[(38, 77)];
    groebner_matrix[(38, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 143)] -= reduction_term;
    let factor = groebner_matrix[(38, 78)];
    groebner_matrix[(38, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 160)] -= reduction_term;
    let factor = -groebner_matrix[(38, 79)];
    groebner_matrix[(38, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 148)] -= reduction_term;
    let factor = -groebner_matrix[(38, 80)];
    groebner_matrix[(38, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 152)] -= reduction_term;
    let factor = -groebner_matrix[(38, 81)];
    groebner_matrix[(38, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 157)] -= reduction_term;
    let factor = groebner_matrix[(38, 82)];
    groebner_matrix[(38, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(38, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(38, 127)] -= reduction_term;
    let factor = groebner_matrix[(38, 83)];
    groebner_matrix[(38, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(38, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(38, 175)] -= reduction_term;
    let factor = groebner_matrix[(38, 84)];
    groebner_matrix[(38, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 166)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_89(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(38, 85)];

    groebner_matrix[(38, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(38, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(38, 130)] -= reduction_term;
    let factor = -groebner_matrix[(38, 86)];
    groebner_matrix[(38, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(38, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(38, 172)] -= reduction_term;
    let factor = -groebner_matrix[(38, 87)];
    groebner_matrix[(38, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 163)] -= reduction_term;
    let factor = groebner_matrix[(38, 91)];
    groebner_matrix[(38, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 173)] -= reduction_term;
    let factor = -groebner_matrix[(38, 94)];
    groebner_matrix[(38, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 170)] -= reduction_term;
    let factor = groebner_matrix[(38, 97)];
    groebner_matrix[(38, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(38, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(38, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(38, 195)] -= reduction_term;
    let factor = groebner_matrix[(38, 100)];
    groebner_matrix[(38, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(38, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(38, 139)] -= reduction_term;
    let factor = groebner_matrix[(38, 103)];
    groebner_matrix[(38, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(38, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(38, 114)] -= reduction_term;
    let factor = -groebner_matrix[(38, 104)];
    groebner_matrix[(38, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(38, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(38, 186)] -= reduction_term;
    let factor = groebner_matrix[(38, 105)];
    groebner_matrix[(38, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(38, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(38, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_90(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(38, 106)];

    groebner_matrix[(38, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(38, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(38, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(38, 195)] -= reduction_term;
    let factor = -groebner_matrix[(38, 109)];
    groebner_matrix[(38, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 184)] -= reduction_term;
    let factor = groebner_matrix[(38, 118)];
    groebner_matrix[(38, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(38, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(38, 135)] -= reduction_term;
    let factor = groebner_matrix[(38, 119)];
    groebner_matrix[(38, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(38, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(38, 183)] -= reduction_term;
    let factor = -groebner_matrix[(38, 120)];
    groebner_matrix[(38, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(38, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(38, 182)] -= reduction_term;
    let factor = groebner_matrix[(38, 121)];
    groebner_matrix[(38, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(38, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(38, 138)] -= reduction_term;
    let factor = -groebner_matrix[(38, 122)];
    groebner_matrix[(38, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(38, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(38, 180)] -= reduction_term;
    let factor = groebner_matrix[(38, 123)];
    groebner_matrix[(38, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(38, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(38, 179)] -= reduction_term;
    let factor = groebner_matrix[(38, 127)];
    groebner_matrix[(38, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 181)] -= reduction_term;
    let factor = -groebner_matrix[(38, 130)];
    groebner_matrix[(38, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 178)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_91(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(38, 142)];

    groebner_matrix[(38, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(38, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(38, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(38, 196)] -= reduction_term;
    let factor = groebner_matrix[(38, 143)];
    groebner_matrix[(38, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(38, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(38, 176)] -= reduction_term;
    let factor = groebner_matrix[(38, 144)];
    groebner_matrix[(38, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(38, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(38, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(38, 196)] -= reduction_term;
    let factor = groebner_matrix[(38, 146)];
    groebner_matrix[(38, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(38, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(38, 185)] -= reduction_term;
    let factor = groebner_matrix[(38, 147)];
    groebner_matrix[(38, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(38, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(38, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(38, 196)] -= reduction_term;
    let factor = groebner_matrix[(38, 148)];
    groebner_matrix[(38, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(38, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(38, 159)] -= reduction_term;
    let factor = -groebner_matrix[(38, 149)];
    groebner_matrix[(38, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(38, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(38, 195)] -= reduction_term;
    let factor = groebner_matrix[(38, 150)];
    groebner_matrix[(38, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(38, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(38, 194)] -= reduction_term;
    let factor = groebner_matrix[(38, 151)];
    groebner_matrix[(38, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(38, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(38, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(38, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 38);
}

#[inline]
fn compute_basis_stage_92(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row31_000000000_f(groebner_matrix, 38);
    let factor = -groebner_matrix[(38, 154)];
    groebner_matrix[(38, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(38, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(38, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 38);
    groebner_row33_000000000_f(groebner_matrix, 38);
    groebner_row34_000000000_f(groebner_matrix, 38);
    groebner_row35_000000000_f(groebner_matrix, 38);
    groebner_row36_000000000_f(groebner_matrix, 38);
    groebner_row37_000000000_f(groebner_matrix, 38);
    let factor = groebner_matrix[(38, 163)];
    groebner_matrix[(38, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(38, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(38, 180)] -= reduction_term;
    let factor = groebner_matrix[(38, 164)];
    groebner_matrix[(38, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(38, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(38, 192)] -= reduction_term;
    let factor = -groebner_matrix[(38, 165)];
    groebner_matrix[(38, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(38, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(38, 191)] -= reduction_term;
    let factor = groebner_matrix[(38, 166)];
    groebner_matrix[(38, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(38, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(38, 183)] -= reduction_term;
    let factor = -groebner_matrix[(38, 167)];
    groebner_matrix[(38, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(38, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(38, 189)] -= reduction_term;
    let factor = groebner_matrix[(38, 168)];
    groebner_matrix[(38, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(38, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(38, 188)] -= reduction_term;
    let factor = groebner_matrix[(38, 169)];
    groebner_matrix[(38, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(38, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(38, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(38, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_93(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(38, 172)];

    groebner_matrix[(38, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(38, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(38, 190)] -= reduction_term;
    let factor = -groebner_matrix[(38, 175)];
    groebner_matrix[(38, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(38, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(38, 187)] -= reduction_term;
    s_polynomial39(groebner_matrix);
    let factor = -groebner_matrix[(39, 1)];
    groebner_matrix[(39, 1)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(39, 8)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(39, 179)] -= reduction_term;
    let factor = groebner_matrix[(39, 2)];
    groebner_matrix[(39, 2)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(39, 11)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(39, 24)] -= reduction_term;
    let factor = -groebner_matrix[(39, 3)];
    groebner_matrix[(39, 3)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(39, 10)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(39, 180)] -= reduction_term;
    let factor = groebner_matrix[(39, 4)];
    groebner_matrix[(39, 4)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(39, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(39, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(39, 190)] -= reduction_term;
    let factor = groebner_matrix[(39, 5)];
    groebner_matrix[(39, 5)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(39, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(39, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(39, 188)] -= reduction_term;
    let factor = groebner_matrix[(39, 6)];
    groebner_matrix[(39, 6)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(39, 18)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(39, 35)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(39, 189)] -= reduction_term;
    let factor = groebner_matrix[(39, 7)];
    groebner_matrix[(39, 7)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(39, 9)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(39, 12)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(39, 191)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_94(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(39, 10)];
    groebner_matrix[(39, 10)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 20)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 163)] -= reduction_term;
    let factor = -groebner_matrix[(39, 11)];
    groebner_matrix[(39, 11)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 164)] -= reduction_term;
    let factor = -groebner_matrix[(39, 12)];
    groebner_matrix[(39, 12)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 23)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 165)] -= reduction_term;
    let factor = groebner_matrix[(39, 13)];
    groebner_matrix[(39, 13)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(39, 17)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(39, 30)] -= reduction_term;
    let factor = -groebner_matrix[(39, 14)];
    groebner_matrix[(39, 14)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(39, 16)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(39, 182)] -= reduction_term;
    let factor = -groebner_matrix[(39, 15)];
    groebner_matrix[(39, 15)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 26)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 166)] -= reduction_term;
    let factor = -groebner_matrix[(39, 18)];
    groebner_matrix[(39, 18)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 167)] -= reduction_term;
    let factor = groebner_matrix[(39, 19)];
    groebner_matrix[(39, 19)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(39, 21)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(39, 24)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(39, 192)] -= reduction_term;
    let factor = groebner_matrix[(39, 22)];
    groebner_matrix[(39, 22)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(39, 36)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(39, 123)] -= reduction_term;
    let factor = groebner_matrix[(39, 25)];
    groebner_matrix[(39, 25)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(39, 29)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(39, 35)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_95(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(39, 26)];
    groebner_matrix[(39, 26)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(39, 28)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(39, 183)] -= reduction_term;
    let factor = groebner_matrix[(39, 27)];
    groebner_matrix[(39, 27)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(39, 33)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(39, 175)] -= reduction_term;
    let factor = -groebner_matrix[(39, 30)];
    groebner_matrix[(39, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 168)] -= reduction_term;
    let factor = groebner_matrix[(39, 39)];
    groebner_matrix[(39, 39)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(39, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(39, 101)] -= reduction_term;
    let factor = groebner_matrix[(39, 40)];
    groebner_matrix[(39, 40)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(39, 62)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(39, 158)] -= reduction_term;
    let factor = groebner_matrix[(39, 41)];
    groebner_matrix[(39, 41)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(39, 65)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(39, 102)] -= reduction_term;
    let factor = groebner_matrix[(39, 42)];
    groebner_matrix[(39, 42)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(39, 64)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(39, 159)] -= reduction_term;
    let factor = groebner_matrix[(39, 43)];
    groebner_matrix[(39, 43)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(39, 56)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(39, 139)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(39, 193)] -= reduction_term;
    let factor = groebner_matrix[(39, 44)];
    groebner_matrix[(39, 44)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(39, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(39, 113)] -= reduction_term;
    let factor = groebner_matrix[(39, 45)];
    groebner_matrix[(39, 45)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(39, 73)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(39, 114)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_96(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(39, 46)];
    groebner_matrix[(39, 46)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(39, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(39, 109)] -= reduction_term;
    let factor = groebner_matrix[(39, 47)];
    groebner_matrix[(39, 47)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(39, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(39, 161)] -= reduction_term;
    let factor = -groebner_matrix[(39, 48)];
    groebner_matrix[(39, 48)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 52)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 169)] -= reduction_term;
    let factor = groebner_matrix[(39, 49)];
    groebner_matrix[(39, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(39, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(39, 116)] -= reduction_term;
    let factor = -groebner_matrix[(39, 50)];
    groebner_matrix[(39, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(39, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(39, 154)] -= reduction_term;
    let factor = groebner_matrix[(39, 51)];
    groebner_matrix[(39, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(39, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(39, 114)] -= reduction_term;
    let factor = groebner_matrix[(39, 52)];
    groebner_matrix[(39, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(39, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(39, 162)] -= reduction_term;
    let factor = groebner_matrix[(39, 53)];
    groebner_matrix[(39, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(39, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(39, 146)] -= reduction_term;
    let factor = groebner_matrix[(39, 54)];
    groebner_matrix[(39, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(39, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(39, 117)] -= reduction_term;
    let factor = -groebner_matrix[(39, 55)];
    groebner_matrix[(39, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(39, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(39, 159)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_97(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(39, 56)];

    groebner_matrix[(39, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(39, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(39, 158)] -= reduction_term;
    let factor = groebner_matrix[(39, 57)];
    groebner_matrix[(39, 57)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(39, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(39, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(39, 188)] -= reduction_term;
    let factor = groebner_matrix[(39, 58)];
    groebner_matrix[(39, 58)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(39, 91)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(39, 135)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(39, 189)] -= reduction_term;
    let factor = groebner_matrix[(39, 59)];
    groebner_matrix[(39, 59)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(39, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(39, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(39, 191)] -= reduction_term;
    let factor = groebner_matrix[(39, 60)];
    groebner_matrix[(39, 60)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(39, 94)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(39, 138)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(39, 192)] -= reduction_term;
    let factor = groebner_matrix[(39, 61)];
    groebner_matrix[(39, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(39, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(39, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(39, 194)] -= reduction_term;
    let factor = groebner_matrix[(39, 64)];
    groebner_matrix[(39, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 148)] -= reduction_term;
    let factor = groebner_matrix[(39, 65)];
    groebner_matrix[(39, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 149)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_98(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(39, 66)];

    groebner_matrix[(39, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 150)] -= reduction_term;
    let factor = groebner_matrix[(39, 67)];
    groebner_matrix[(39, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(39, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(39, 78)] -= reduction_term;
    let factor = -groebner_matrix[(39, 68)];
    groebner_matrix[(39, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(39, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(39, 185)] -= reduction_term;
    let factor = groebner_matrix[(39, 69)];
    groebner_matrix[(39, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 151)] -= reduction_term;
    let factor = groebner_matrix[(39, 70)];
    groebner_matrix[(39, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(39, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(39, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(39, 194)] -= reduction_term;
    let factor = -groebner_matrix[(39, 73)];
    groebner_matrix[(39, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 176)] -= reduction_term;
    let factor = -groebner_matrix[(39, 76)];
    groebner_matrix[(39, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 142)] -= reduction_term;
    let factor = -groebner_matrix[(39, 77)];
    groebner_matrix[(39, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 143)] -= reduction_term;
    let factor = groebner_matrix[(39, 78)];
    groebner_matrix[(39, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 160)] -= reduction_term;
    let factor = -groebner_matrix[(39, 79)];
    groebner_matrix[(39, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 148)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_99(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(39, 80)];

    groebner_matrix[(39, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 152)] -= reduction_term;
    let factor = -groebner_matrix[(39, 81)];
    groebner_matrix[(39, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 157)] -= reduction_term;
    let factor = groebner_matrix[(39, 82)];
    groebner_matrix[(39, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(39, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(39, 127)] -= reduction_term;
    let factor = groebner_matrix[(39, 83)];
    groebner_matrix[(39, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(39, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(39, 175)] -= reduction_term;
    let factor = groebner_matrix[(39, 84)];
    groebner_matrix[(39, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 166)] -= reduction_term;
    let factor = groebner_matrix[(39, 85)];
    groebner_matrix[(39, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(39, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(39, 130)] -= reduction_term;
    let factor = -groebner_matrix[(39, 86)];
    groebner_matrix[(39, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(39, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(39, 172)] -= reduction_term;
    let factor = -groebner_matrix[(39, 87)];
    groebner_matrix[(39, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 163)] -= reduction_term;
    let factor = groebner_matrix[(39, 91)];
    groebner_matrix[(39, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 173)] -= reduction_term;
    let factor = -groebner_matrix[(39, 94)];
    groebner_matrix[(39, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 170)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_100(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(39, 97)];

    groebner_matrix[(39, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(39, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(39, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(39, 195)] -= reduction_term;
    let factor = groebner_matrix[(39, 100)];
    groebner_matrix[(39, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(39, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(39, 139)] -= reduction_term;
    let factor = groebner_matrix[(39, 103)];
    groebner_matrix[(39, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(39, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(39, 114)] -= reduction_term;
    let factor = -groebner_matrix[(39, 104)];
    groebner_matrix[(39, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(39, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(39, 186)] -= reduction_term;
    let factor = groebner_matrix[(39, 105)];
    groebner_matrix[(39, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(39, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(39, 185)] -= reduction_term;
    let factor = groebner_matrix[(39, 106)];
    groebner_matrix[(39, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(39, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(39, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(39, 195)] -= reduction_term;
    let factor = -groebner_matrix[(39, 109)];
    groebner_matrix[(39, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 184)] -= reduction_term;
    let factor = groebner_matrix[(39, 118)];
    groebner_matrix[(39, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(39, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(39, 135)] -= reduction_term;
    let factor = groebner_matrix[(39, 119)];
    groebner_matrix[(39, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(39, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(39, 183)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_101(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(39, 120)];

    groebner_matrix[(39, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(39, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(39, 182)] -= reduction_term;
    let factor = groebner_matrix[(39, 121)];
    groebner_matrix[(39, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(39, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(39, 138)] -= reduction_term;
    let factor = -groebner_matrix[(39, 122)];
    groebner_matrix[(39, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(39, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(39, 180)] -= reduction_term;
    let factor = groebner_matrix[(39, 123)];
    groebner_matrix[(39, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(39, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(39, 179)] -= reduction_term;
    let factor = groebner_matrix[(39, 127)];
    groebner_matrix[(39, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 181)] -= reduction_term;
    let factor = -groebner_matrix[(39, 130)];
    groebner_matrix[(39, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 178)] -= reduction_term;
    let factor = groebner_matrix[(39, 142)];
    groebner_matrix[(39, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(39, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(39, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(39, 196)] -= reduction_term;
    let factor = groebner_matrix[(39, 143)];
    groebner_matrix[(39, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(39, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(39, 176)] -= reduction_term;
    let factor = groebner_matrix[(39, 144)];
    groebner_matrix[(39, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(39, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(39, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(39, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_102(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(39, 146)];

    groebner_matrix[(39, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(39, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(39, 185)] -= reduction_term;
    let factor = groebner_matrix[(39, 147)];
    groebner_matrix[(39, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(39, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(39, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(39, 196)] -= reduction_term;
    let factor = groebner_matrix[(39, 148)];
    groebner_matrix[(39, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(39, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(39, 159)] -= reduction_term;
    let factor = -groebner_matrix[(39, 149)];
    groebner_matrix[(39, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(39, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(39, 195)] -= reduction_term;
    let factor = groebner_matrix[(39, 150)];
    groebner_matrix[(39, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(39, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(39, 194)] -= reduction_term;
    let factor = groebner_matrix[(39, 151)];
    groebner_matrix[(39, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(39, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(39, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(39, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 39);
    groebner_row31_000000000_f(groebner_matrix, 39);
    let factor = -groebner_matrix[(39, 154)];
    groebner_matrix[(39, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(39, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(39, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 39);
    groebner_row33_000000000_f(groebner_matrix, 39);
    groebner_row34_000000000_f(groebner_matrix, 39);
    groebner_row35_000000000_f(groebner_matrix, 39);
    groebner_row36_000000000_f(groebner_matrix, 39);
    groebner_row37_000000000_f(groebner_matrix, 39);
    groebner_row38_000000000_f(groebner_matrix, 39);
    let factor = groebner_matrix[(39, 163)];
    groebner_matrix[(39, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(39, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(39, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_103(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(39, 164)];

    groebner_matrix[(39, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(39, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(39, 192)] -= reduction_term;
    let factor = -groebner_matrix[(39, 165)];
    groebner_matrix[(39, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(39, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(39, 191)] -= reduction_term;
    let factor = groebner_matrix[(39, 166)];
    groebner_matrix[(39, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(39, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(39, 183)] -= reduction_term;
    let factor = -groebner_matrix[(39, 167)];
    groebner_matrix[(39, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(39, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(39, 189)] -= reduction_term;
    let factor = groebner_matrix[(39, 168)];
    groebner_matrix[(39, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(39, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(39, 188)] -= reduction_term;
    let factor = groebner_matrix[(39, 169)];
    groebner_matrix[(39, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(39, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(39, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(39, 196)] -= reduction_term;
    let factor = groebner_matrix[(39, 172)];
    groebner_matrix[(39, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(39, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(39, 190)] -= reduction_term;
    let factor = -groebner_matrix[(39, 175)];
    groebner_matrix[(39, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(39, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(39, 187)] -= reduction_term;
    s_polynomial40(groebner_matrix);
    groebner_row38_100000000_f(groebner_matrix, 40);
    groebner_row39_100000000_f(groebner_matrix, 40);
    let factor = groebner_matrix[(40, 127)];
    groebner_matrix[(40, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(40, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(40, 181)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_104(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(40, 130)];

    groebner_matrix[(40, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(40, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(40, 178)] -= reduction_term;
    groebner_row34_000000000_f(groebner_matrix, 40);
    groebner_row35_000000000_f(groebner_matrix, 40);
    groebner_row36_000000000_f(groebner_matrix, 40);
    groebner_row37_000000000_f(groebner_matrix, 40);
    groebner_row38_000000000_f(groebner_matrix, 40);
    groebner_row39_000000000_f(groebner_matrix, 40);
    let factor = groebner_matrix[(40, 172)];
    groebner_matrix[(40, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(40, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(40, 190)] -= reduction_term;
    let factor = -groebner_matrix[(40, 175)];
    groebner_matrix[(40, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(40, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(40, 187)] -= reduction_term;
    s_polynomial41(groebner_matrix);
    let factor = -groebner_matrix[(41, 81)];
    groebner_matrix[(41, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(41, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(41, 157)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 41);
    groebner_row33_100000000_f(groebner_matrix, 41);
    groebner_row34_100000000_f(groebner_matrix, 41);
    groebner_row35_100000000_f(groebner_matrix, 41);
    groebner_row36_100000000_f(groebner_matrix, 41);
    groebner_row37_100000000_f(groebner_matrix, 41);
    groebner_row38_100000000_f(groebner_matrix, 41);
    groebner_row39_100000000_f(groebner_matrix, 41);
    let factor = groebner_matrix[(41, 127)];
    groebner_matrix[(41, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(41, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(41, 181)] -= reduction_term;
    let factor = -groebner_matrix[(41, 130)];
    groebner_matrix[(41, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(41, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(41, 178)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 41);
    groebner_row31_000000000_f(groebner_matrix, 41);
    groebner_row32_000000000_f(groebner_matrix, 41);
    groebner_row33_000000000_f(groebner_matrix, 41);
    groebner_row34_000000000_f(groebner_matrix, 41);
    groebner_row35_000000000_f(groebner_matrix, 41);
    groebner_row36_000000000_f(groebner_matrix, 41);
    groebner_row37_000000000_f(groebner_matrix, 41);
    groebner_row38_000000000_f(groebner_matrix, 41);
    groebner_row39_000000000_f(groebner_matrix, 41);
}

#[inline]
fn compute_basis_stage_105(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(41, 172)];

    groebner_matrix[(41, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(41, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(41, 190)] -= reduction_term;
    let factor = -groebner_matrix[(41, 175)];
    groebner_matrix[(41, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(41, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(41, 187)] -= reduction_term;
    s_polynomial42(groebner_matrix);
    let factor = -groebner_matrix[(42, 80)];
    groebner_matrix[(42, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(42, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(42, 152)] -= reduction_term;
    let factor = -groebner_matrix[(42, 81)];
    groebner_matrix[(42, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(42, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(42, 157)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 42);
    groebner_row41_000000000_f(groebner_matrix, 42);
    groebner_row32_100000000_f(groebner_matrix, 42);
    groebner_row33_100000000_f(groebner_matrix, 42);
    groebner_row34_100000000_f(groebner_matrix, 42);
    groebner_row35_100000000_f(groebner_matrix, 42);
    groebner_row36_100000000_f(groebner_matrix, 42);
    groebner_row37_100000000_f(groebner_matrix, 42);
    groebner_row38_100000000_f(groebner_matrix, 42);
    groebner_row39_100000000_f(groebner_matrix, 42);
    let factor = groebner_matrix[(42, 127)];
    groebner_matrix[(42, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(42, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(42, 181)] -= reduction_term;
    let factor = -groebner_matrix[(42, 130)];
    groebner_matrix[(42, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(42, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(42, 178)] -= reduction_term;
    let factor = groebner_matrix[(42, 148)];
    groebner_matrix[(42, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(42, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(42, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 42);
    groebner_row31_000000000_f(groebner_matrix, 42);
    groebner_row32_000000000_f(groebner_matrix, 42);
    groebner_row33_000000000_f(groebner_matrix, 42);
    groebner_row34_000000000_f(groebner_matrix, 42);
    groebner_row35_000000000_f(groebner_matrix, 42);
    groebner_row36_000000000_f(groebner_matrix, 42);
    groebner_row37_000000000_f(groebner_matrix, 42);
    groebner_row38_000000000_f(groebner_matrix, 42);
    groebner_row39_000000000_f(groebner_matrix, 42);
}

#[inline]
fn compute_basis_stage_106(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(42, 172)];
    groebner_matrix[(42, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(42, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(42, 190)] -= reduction_term;
    let factor = -groebner_matrix[(42, 175)];
    groebner_matrix[(42, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(42, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(42, 187)] -= reduction_term;
    s_polynomial43(groebner_matrix);
    let factor = -groebner_matrix[(43, 79)];
    groebner_matrix[(43, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(43, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(43, 148)] -= reduction_term;
    let factor = -groebner_matrix[(43, 80)];
    groebner_matrix[(43, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(43, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(43, 152)] -= reduction_term;
    let factor = -groebner_matrix[(43, 81)];
    groebner_matrix[(43, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(43, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(43, 157)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 43);
    groebner_row41_000000000_f(groebner_matrix, 43);
    groebner_row42_000000000_f(groebner_matrix, 43);
    groebner_row32_100000000_f(groebner_matrix, 43);
    groebner_row33_100000000_f(groebner_matrix, 43);
    groebner_row34_100000000_f(groebner_matrix, 43);
    groebner_row35_100000000_f(groebner_matrix, 43);
    groebner_row36_100000000_f(groebner_matrix, 43);
    groebner_row37_100000000_f(groebner_matrix, 43);
    groebner_row38_100000000_f(groebner_matrix, 43);
    groebner_row39_100000000_f(groebner_matrix, 43);
    let factor = groebner_matrix[(43, 127)];
    groebner_matrix[(43, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(43, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(43, 181)] -= reduction_term;
    let factor = -groebner_matrix[(43, 130)];
    groebner_matrix[(43, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(43, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(43, 178)] -= reduction_term;
    let factor = groebner_matrix[(43, 148)];
    groebner_matrix[(43, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(43, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(43, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 43);
    groebner_row31_000000000_f(groebner_matrix, 43);
}

#[inline]
fn compute_basis_stage_107(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row32_000000000_f(groebner_matrix, 43);
    groebner_row33_000000000_f(groebner_matrix, 43);
    groebner_row34_000000000_f(groebner_matrix, 43);
    groebner_row35_000000000_f(groebner_matrix, 43);
    groebner_row36_000000000_f(groebner_matrix, 43);
    groebner_row37_000000000_f(groebner_matrix, 43);
    groebner_row38_000000000_f(groebner_matrix, 43);
    groebner_row39_000000000_f(groebner_matrix, 43);
    let factor = groebner_matrix[(43, 172)];
    groebner_matrix[(43, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(43, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(43, 190)] -= reduction_term;
    let factor = -groebner_matrix[(43, 175)];
    groebner_matrix[(43, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(43, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(43, 187)] -= reduction_term;
    s_polynomial44(groebner_matrix);
    let factor = groebner_matrix[(44, 78)];
    groebner_matrix[(44, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(44, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(44, 160)] -= reduction_term;
    let factor = -groebner_matrix[(44, 79)];
    groebner_matrix[(44, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(44, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(44, 148)] -= reduction_term;
    let factor = -groebner_matrix[(44, 80)];
    groebner_matrix[(44, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(44, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(44, 152)] -= reduction_term;
    let factor = -groebner_matrix[(44, 81)];
    groebner_matrix[(44, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(44, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(44, 157)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 44);
    groebner_row41_000000000_f(groebner_matrix, 44);
    groebner_row42_000000000_f(groebner_matrix, 44);
    groebner_row43_000000000_f(groebner_matrix, 44);
    groebner_row31_100000000_f(groebner_matrix, 44);
    groebner_row32_100000000_f(groebner_matrix, 44);
    groebner_row33_100000000_f(groebner_matrix, 44);
    groebner_row34_100000000_f(groebner_matrix, 44);
    groebner_row35_100000000_f(groebner_matrix, 44);
    groebner_row36_100000000_f(groebner_matrix, 44);
    groebner_row37_100000000_f(groebner_matrix, 44);
    groebner_row38_100000000_f(groebner_matrix, 44);
    groebner_row39_100000000_f(groebner_matrix, 44);
}

#[inline]
fn compute_basis_stage_108(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(44, 127)];

    groebner_matrix[(44, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(44, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(44, 181)] -= reduction_term;
    let factor = -groebner_matrix[(44, 130)];
    groebner_matrix[(44, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(44, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(44, 178)] -= reduction_term;
    let factor = groebner_matrix[(44, 143)];
    groebner_matrix[(44, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(44, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(44, 176)] -= reduction_term;
    let factor = groebner_matrix[(44, 148)];
    groebner_matrix[(44, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(44, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(44, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 44);
    groebner_row31_000000000_f(groebner_matrix, 44);
    groebner_row32_000000000_f(groebner_matrix, 44);
    groebner_row33_000000000_f(groebner_matrix, 44);
    groebner_row34_000000000_f(groebner_matrix, 44);
    groebner_row35_000000000_f(groebner_matrix, 44);
    groebner_row36_000000000_f(groebner_matrix, 44);
    groebner_row37_000000000_f(groebner_matrix, 44);
    groebner_row38_000000000_f(groebner_matrix, 44);
    groebner_row39_000000000_f(groebner_matrix, 44);
    let factor = groebner_matrix[(44, 172)];
    groebner_matrix[(44, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(44, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(44, 190)] -= reduction_term;
    let factor = -groebner_matrix[(44, 175)];
    groebner_matrix[(44, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(44, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(44, 187)] -= reduction_term;
    s_polynomial45(groebner_matrix);
    let factor = -groebner_matrix[(45, 77)];
    groebner_matrix[(45, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(45, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(45, 143)] -= reduction_term;
    let factor = groebner_matrix[(45, 78)];
    groebner_matrix[(45, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(45, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(45, 160)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_109(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(45, 79)];

    groebner_matrix[(45, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(45, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(45, 148)] -= reduction_term;
    let factor = -groebner_matrix[(45, 80)];
    groebner_matrix[(45, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(45, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(45, 152)] -= reduction_term;
    let factor = -groebner_matrix[(45, 81)];
    groebner_matrix[(45, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(45, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(45, 157)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 45);
    groebner_row41_000000000_f(groebner_matrix, 45);
    groebner_row42_000000000_f(groebner_matrix, 45);
    groebner_row43_000000000_f(groebner_matrix, 45);
    groebner_row44_000000000_f(groebner_matrix, 45);
    groebner_row30_100000000_f(groebner_matrix, 45);
    groebner_row31_100000000_f(groebner_matrix, 45);
    groebner_row32_100000000_f(groebner_matrix, 45);
    groebner_row33_100000000_f(groebner_matrix, 45);
    groebner_row34_100000000_f(groebner_matrix, 45);
    groebner_row35_100000000_f(groebner_matrix, 45);
    groebner_row36_100000000_f(groebner_matrix, 45);
    groebner_row37_100000000_f(groebner_matrix, 45);
    groebner_row38_100000000_f(groebner_matrix, 45);
    groebner_row39_100000000_f(groebner_matrix, 45);
    let factor = groebner_matrix[(45, 127)];
    groebner_matrix[(45, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(45, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(45, 181)] -= reduction_term;
    let factor = -groebner_matrix[(45, 130)];
    groebner_matrix[(45, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(45, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(45, 178)] -= reduction_term;
    let factor = groebner_matrix[(45, 142)];
    groebner_matrix[(45, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(45, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(45, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(45, 196)] -= reduction_term;
    let factor = groebner_matrix[(45, 143)];
    groebner_matrix[(45, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(45, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(45, 176)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_110(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(45, 144)];

    groebner_matrix[(45, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(45, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(45, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(45, 196)] -= reduction_term;
    let factor = groebner_matrix[(45, 147)];
    groebner_matrix[(45, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(45, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(45, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(45, 196)] -= reduction_term;
    let factor = groebner_matrix[(45, 148)];
    groebner_matrix[(45, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(45, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(45, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 45);
    groebner_row31_000000000_f(groebner_matrix, 45);
    groebner_row32_000000000_f(groebner_matrix, 45);
    groebner_row33_000000000_f(groebner_matrix, 45);
    groebner_row34_000000000_f(groebner_matrix, 45);
    groebner_row35_000000000_f(groebner_matrix, 45);
    groebner_row36_000000000_f(groebner_matrix, 45);
    groebner_row37_000000000_f(groebner_matrix, 45);
    groebner_row38_000000000_f(groebner_matrix, 45);
    groebner_row39_000000000_f(groebner_matrix, 45);
    let factor = groebner_matrix[(45, 172)];
    groebner_matrix[(45, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(45, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(45, 190)] -= reduction_term;
    let factor = -groebner_matrix[(45, 175)];
    groebner_matrix[(45, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(45, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(45, 187)] -= reduction_term;
    s_polynomial46(groebner_matrix);
    let factor = groebner_matrix[(46, 82)];
    groebner_matrix[(46, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(46, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(46, 127)] -= reduction_term;
    let factor = groebner_matrix[(46, 83)];
    groebner_matrix[(46, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(46, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(46, 175)] -= reduction_term;
    let factor = groebner_matrix[(46, 85)];
    groebner_matrix[(46, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(46, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(46, 130)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_111(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(46, 86)];
    groebner_matrix[(46, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(46, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(46, 172)] -= reduction_term;
    let factor = groebner_matrix[(46, 88)];
    groebner_matrix[(46, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(46, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(46, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(46, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 46);
    groebner_row41_000000000_f(groebner_matrix, 46);
    groebner_row42_000000000_f(groebner_matrix, 46);
    groebner_row43_000000000_f(groebner_matrix, 46);
    groebner_row44_000000000_f(groebner_matrix, 46);
    groebner_row45_000000000_f(groebner_matrix, 46);
    groebner_row37_100000000_f(groebner_matrix, 46);
    groebner_row38_100000000_f(groebner_matrix, 46);
    groebner_row39_100000000_f(groebner_matrix, 46);
    let factor = groebner_matrix[(46, 118)];
    groebner_matrix[(46, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(46, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(46, 135)] -= reduction_term;
    let factor = groebner_matrix[(46, 119)];
    groebner_matrix[(46, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(46, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(46, 183)] -= reduction_term;
    let factor = -groebner_matrix[(46, 120)];
    groebner_matrix[(46, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(46, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(46, 182)] -= reduction_term;
    let factor = groebner_matrix[(46, 121)];
    groebner_matrix[(46, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(46, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(46, 138)] -= reduction_term;
    let factor = -groebner_matrix[(46, 122)];
    groebner_matrix[(46, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(46, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(46, 180)] -= reduction_term;
    let factor = groebner_matrix[(46, 123)];
    groebner_matrix[(46, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(46, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(46, 179)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_112(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(46, 124)];

    groebner_matrix[(46, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(46, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(46, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(46, 195)] -= reduction_term;
    let factor = groebner_matrix[(46, 127)];
    groebner_matrix[(46, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(46, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(46, 181)] -= reduction_term;
    let factor = -groebner_matrix[(46, 130)];
    groebner_matrix[(46, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(46, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(46, 178)] -= reduction_term;
    groebner_row35_000000000_f(groebner_matrix, 46);
    groebner_row36_000000000_f(groebner_matrix, 46);
    groebner_row37_000000000_f(groebner_matrix, 46);
    groebner_row38_000000000_f(groebner_matrix, 46);
    groebner_row39_000000000_f(groebner_matrix, 46);
    let factor = groebner_matrix[(46, 163)];
    groebner_matrix[(46, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(46, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(46, 180)] -= reduction_term;
    let factor = groebner_matrix[(46, 164)];
    groebner_matrix[(46, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(46, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(46, 192)] -= reduction_term;
    let factor = -groebner_matrix[(46, 165)];
    groebner_matrix[(46, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(46, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(46, 191)] -= reduction_term;
    let factor = groebner_matrix[(46, 166)];
    groebner_matrix[(46, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(46, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(46, 183)] -= reduction_term;
    let factor = -groebner_matrix[(46, 167)];
    groebner_matrix[(46, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(46, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(46, 189)] -= reduction_term;
    let factor = groebner_matrix[(46, 168)];
    groebner_matrix[(46, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(46, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(46, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_113(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(46, 169)];

    groebner_matrix[(46, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(46, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(46, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(46, 196)] -= reduction_term;
    let factor = groebner_matrix[(46, 172)];
    groebner_matrix[(46, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(46, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(46, 190)] -= reduction_term;
    let factor = -groebner_matrix[(46, 175)];
    groebner_matrix[(46, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(46, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(46, 187)] -= reduction_term;
    s_polynomial47(groebner_matrix);
    let factor = groebner_matrix[(47, 56)];
    groebner_matrix[(47, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(47, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(47, 158)] -= reduction_term;
    let factor = -groebner_matrix[(47, 79)];
    groebner_matrix[(47, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(47, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(47, 148)] -= reduction_term;
    let factor = groebner_matrix[(47, 82)];
    groebner_matrix[(47, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(47, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(47, 127)] -= reduction_term;
    let factor = groebner_matrix[(47, 83)];
    groebner_matrix[(47, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(47, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(47, 175)] -= reduction_term;
    let factor = groebner_matrix[(47, 85)];
    groebner_matrix[(47, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(47, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(47, 130)] -= reduction_term;
    let factor = -groebner_matrix[(47, 86)];
    groebner_matrix[(47, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(47, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(47, 172)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_114(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(47, 88)];

    groebner_matrix[(47, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(47, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(47, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(47, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 47);
    groebner_row41_000000000_f(groebner_matrix, 47);
    groebner_row42_000000000_f(groebner_matrix, 47);
    groebner_row43_000000000_f(groebner_matrix, 47);
    groebner_row44_000000000_f(groebner_matrix, 47);
    groebner_row45_000000000_f(groebner_matrix, 47);
    groebner_row32_100000000_f(groebner_matrix, 47);
    groebner_row33_100000000_f(groebner_matrix, 47);
    groebner_row34_100000000_f(groebner_matrix, 47);
    groebner_row35_100000000_f(groebner_matrix, 47);
    groebner_row36_100000000_f(groebner_matrix, 47);
    groebner_row37_100000000_f(groebner_matrix, 47);
    groebner_row38_100000000_f(groebner_matrix, 47);
    groebner_row39_100000000_f(groebner_matrix, 47);
    let factor = groebner_matrix[(47, 118)];
    groebner_matrix[(47, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(47, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(47, 135)] -= reduction_term;
    let factor = groebner_matrix[(47, 119)];
    groebner_matrix[(47, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(47, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(47, 183)] -= reduction_term;
    let factor = -groebner_matrix[(47, 120)];
    groebner_matrix[(47, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(47, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(47, 182)] -= reduction_term;
    let factor = groebner_matrix[(47, 121)];
    groebner_matrix[(47, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(47, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(47, 138)] -= reduction_term;
    let factor = -groebner_matrix[(47, 122)];
    groebner_matrix[(47, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(47, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(47, 180)] -= reduction_term;
    let factor = groebner_matrix[(47, 123)];
    groebner_matrix[(47, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(47, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(47, 179)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_115(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(47, 124)];

    groebner_matrix[(47, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(47, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(47, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(47, 195)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 47);
    let factor = groebner_matrix[(47, 127)];
    groebner_matrix[(47, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(47, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(47, 181)] -= reduction_term;
    let factor = -groebner_matrix[(47, 130)];
    groebner_matrix[(47, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(47, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(47, 178)] -= reduction_term;
    let factor = groebner_matrix[(47, 148)];
    groebner_matrix[(47, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(47, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(47, 159)] -= reduction_term;
    groebner_row31_000000000_f(groebner_matrix, 47);
    groebner_row32_000000000_f(groebner_matrix, 47);
    groebner_row33_000000000_f(groebner_matrix, 47);
    groebner_row34_000000000_f(groebner_matrix, 47);
    groebner_row35_000000000_f(groebner_matrix, 47);
    groebner_row36_000000000_f(groebner_matrix, 47);
    groebner_row37_000000000_f(groebner_matrix, 47);
    groebner_row38_000000000_f(groebner_matrix, 47);
    groebner_row39_000000000_f(groebner_matrix, 47);
    let factor = groebner_matrix[(47, 163)];
    groebner_matrix[(47, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(47, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(47, 180)] -= reduction_term;
    let factor = groebner_matrix[(47, 164)];
    groebner_matrix[(47, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(47, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(47, 192)] -= reduction_term;
    let factor = -groebner_matrix[(47, 165)];
    groebner_matrix[(47, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(47, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(47, 191)] -= reduction_term;
    let factor = groebner_matrix[(47, 166)];
    groebner_matrix[(47, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(47, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(47, 183)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_116(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(47, 167)];

    groebner_matrix[(47, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(47, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(47, 189)] -= reduction_term;
    let factor = groebner_matrix[(47, 168)];
    groebner_matrix[(47, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(47, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(47, 188)] -= reduction_term;
    let factor = groebner_matrix[(47, 169)];
    groebner_matrix[(47, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(47, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(47, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(47, 196)] -= reduction_term;
    let factor = groebner_matrix[(47, 172)];
    groebner_matrix[(47, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(47, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(47, 190)] -= reduction_term;
    let factor = -groebner_matrix[(47, 175)];
    groebner_matrix[(47, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(47, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(47, 187)] -= reduction_term;
    s_polynomial48(groebner_matrix);
    let factor = -groebner_matrix[(48, 55)];
    groebner_matrix[(48, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(48, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(48, 159)] -= reduction_term;
    let factor = groebner_matrix[(48, 56)];
    groebner_matrix[(48, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(48, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(48, 158)] -= reduction_term;
    let factor = -groebner_matrix[(48, 79)];
    groebner_matrix[(48, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(48, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(48, 148)] -= reduction_term;
    let factor = -groebner_matrix[(48, 80)];
    groebner_matrix[(48, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(48, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(48, 152)] -= reduction_term;
    let factor = groebner_matrix[(48, 82)];
    groebner_matrix[(48, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(48, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(48, 127)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_117(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(48, 83)];
    groebner_matrix[(48, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(48, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(48, 175)] -= reduction_term;
    let factor = groebner_matrix[(48, 85)];
    groebner_matrix[(48, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(48, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(48, 130)] -= reduction_term;
    let factor = -groebner_matrix[(48, 86)];
    groebner_matrix[(48, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(48, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(48, 172)] -= reduction_term;
    let factor = groebner_matrix[(48, 88)];
    groebner_matrix[(48, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(48, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(48, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(48, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 48);
    groebner_row41_000000000_f(groebner_matrix, 48);
    groebner_row42_000000000_f(groebner_matrix, 48);
    groebner_row43_000000000_f(groebner_matrix, 48);
    groebner_row44_000000000_f(groebner_matrix, 48);
    groebner_row45_000000000_f(groebner_matrix, 48);
    groebner_row32_100000000_f(groebner_matrix, 48);
    groebner_row33_100000000_f(groebner_matrix, 48);
    groebner_row34_100000000_f(groebner_matrix, 48);
    groebner_row35_100000000_f(groebner_matrix, 48);
    groebner_row36_100000000_f(groebner_matrix, 48);
    groebner_row37_100000000_f(groebner_matrix, 48);
    groebner_row38_100000000_f(groebner_matrix, 48);
    groebner_row39_100000000_f(groebner_matrix, 48);
    let factor = groebner_matrix[(48, 118)];
    groebner_matrix[(48, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(48, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(48, 135)] -= reduction_term;
    let factor = groebner_matrix[(48, 119)];
    groebner_matrix[(48, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(48, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(48, 183)] -= reduction_term;
    let factor = -groebner_matrix[(48, 120)];
    groebner_matrix[(48, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(48, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(48, 182)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_118(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(48, 121)];

    groebner_matrix[(48, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(48, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(48, 138)] -= reduction_term;
    let factor = -groebner_matrix[(48, 122)];
    groebner_matrix[(48, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(48, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(48, 180)] -= reduction_term;
    let factor = groebner_matrix[(48, 123)];
    groebner_matrix[(48, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(48, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(48, 179)] -= reduction_term;
    let factor = groebner_matrix[(48, 124)];
    groebner_matrix[(48, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(48, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(48, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(48, 195)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 48);
    groebner_row47_000000000_f(groebner_matrix, 48);
    let factor = groebner_matrix[(48, 127)];
    groebner_matrix[(48, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(48, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(48, 181)] -= reduction_term;
    let factor = -groebner_matrix[(48, 130)];
    groebner_matrix[(48, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(48, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(48, 178)] -= reduction_term;
    let factor = groebner_matrix[(48, 148)];
    groebner_matrix[(48, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(48, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(48, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 48);
    groebner_row31_000000000_f(groebner_matrix, 48);
    groebner_row32_000000000_f(groebner_matrix, 48);
    groebner_row33_000000000_f(groebner_matrix, 48);
    groebner_row34_000000000_f(groebner_matrix, 48);
    groebner_row35_000000000_f(groebner_matrix, 48);
    groebner_row36_000000000_f(groebner_matrix, 48);
    groebner_row37_000000000_f(groebner_matrix, 48);
    groebner_row38_000000000_f(groebner_matrix, 48);
    groebner_row39_000000000_f(groebner_matrix, 48);
    let factor = groebner_matrix[(48, 163)];
    groebner_matrix[(48, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(48, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(48, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_119(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(48, 164)];

    groebner_matrix[(48, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(48, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(48, 192)] -= reduction_term;
    let factor = -groebner_matrix[(48, 165)];
    groebner_matrix[(48, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(48, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(48, 191)] -= reduction_term;
    let factor = groebner_matrix[(48, 166)];
    groebner_matrix[(48, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(48, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(48, 183)] -= reduction_term;
    let factor = -groebner_matrix[(48, 167)];
    groebner_matrix[(48, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(48, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(48, 189)] -= reduction_term;
    let factor = groebner_matrix[(48, 168)];
    groebner_matrix[(48, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(48, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(48, 188)] -= reduction_term;
    let factor = groebner_matrix[(48, 169)];
    groebner_matrix[(48, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(48, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(48, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(48, 196)] -= reduction_term;
    let factor = groebner_matrix[(48, 172)];
    groebner_matrix[(48, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(48, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(48, 190)] -= reduction_term;
    let factor = -groebner_matrix[(48, 175)];
    groebner_matrix[(48, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(48, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(48, 187)] -= reduction_term;
    s_polynomial49(groebner_matrix);
    let factor = groebner_matrix[(49, 54)];
    groebner_matrix[(49, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(49, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(49, 117)] -= reduction_term;
    let factor = -groebner_matrix[(49, 55)];
    groebner_matrix[(49, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(49, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(49, 159)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_120(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(49, 56)];
    groebner_matrix[(49, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(49, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(49, 158)] -= reduction_term;
    let factor = -groebner_matrix[(49, 79)];
    groebner_matrix[(49, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(49, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(49, 148)] -= reduction_term;
    let factor = -groebner_matrix[(49, 80)];
    groebner_matrix[(49, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(49, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(49, 152)] -= reduction_term;
    let factor = groebner_matrix[(49, 82)];
    groebner_matrix[(49, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(49, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(49, 127)] -= reduction_term;
    let factor = groebner_matrix[(49, 83)];
    groebner_matrix[(49, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(49, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(49, 175)] -= reduction_term;
    let factor = groebner_matrix[(49, 85)];
    groebner_matrix[(49, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(49, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(49, 130)] -= reduction_term;
    let factor = -groebner_matrix[(49, 86)];
    groebner_matrix[(49, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(49, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(49, 172)] -= reduction_term;
    let factor = groebner_matrix[(49, 88)];
    groebner_matrix[(49, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(49, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(49, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(49, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 49);
    groebner_row41_000000000_f(groebner_matrix, 49);
    groebner_row42_000000000_f(groebner_matrix, 49);
    groebner_row43_000000000_f(groebner_matrix, 49);
    groebner_row44_000000000_f(groebner_matrix, 49);
    groebner_row45_000000000_f(groebner_matrix, 49);
    let factor = groebner_matrix[(49, 105)];
    groebner_matrix[(49, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(49, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(49, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_121(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row32_100000000_f(groebner_matrix, 49);
    groebner_row33_100000000_f(groebner_matrix, 49);
    groebner_row34_100000000_f(groebner_matrix, 49);
    groebner_row35_100000000_f(groebner_matrix, 49);
    groebner_row36_100000000_f(groebner_matrix, 49);
    groebner_row37_100000000_f(groebner_matrix, 49);
    groebner_row38_100000000_f(groebner_matrix, 49);
    groebner_row39_100000000_f(groebner_matrix, 49);
    let factor = groebner_matrix[(49, 118)];
    groebner_matrix[(49, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(49, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(49, 135)] -= reduction_term;
    let factor = groebner_matrix[(49, 119)];
    groebner_matrix[(49, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(49, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(49, 183)] -= reduction_term;
    let factor = -groebner_matrix[(49, 120)];
    groebner_matrix[(49, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(49, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(49, 182)] -= reduction_term;
    let factor = groebner_matrix[(49, 121)];
    groebner_matrix[(49, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(49, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(49, 138)] -= reduction_term;
    let factor = -groebner_matrix[(49, 122)];
    groebner_matrix[(49, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(49, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(49, 180)] -= reduction_term;
    let factor = groebner_matrix[(49, 123)];
    groebner_matrix[(49, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(49, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(49, 179)] -= reduction_term;
    let factor = groebner_matrix[(49, 124)];
    groebner_matrix[(49, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(49, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(49, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(49, 195)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 49);
    groebner_row47_000000000_f(groebner_matrix, 49);
    let factor = groebner_matrix[(49, 127)];
    groebner_matrix[(49, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(49, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(49, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 49);
}

#[inline]
fn compute_basis_stage_122(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(49, 130)];

    groebner_matrix[(49, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(49, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(49, 178)] -= reduction_term;
    let factor = groebner_matrix[(49, 146)];
    groebner_matrix[(49, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(49, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(49, 185)] -= reduction_term;
    let factor = groebner_matrix[(49, 148)];
    groebner_matrix[(49, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(49, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(49, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 49);
    groebner_row31_000000000_f(groebner_matrix, 49);
    groebner_row32_000000000_f(groebner_matrix, 49);
    groebner_row33_000000000_f(groebner_matrix, 49);
    groebner_row34_000000000_f(groebner_matrix, 49);
    groebner_row35_000000000_f(groebner_matrix, 49);
    groebner_row36_000000000_f(groebner_matrix, 49);
    groebner_row37_000000000_f(groebner_matrix, 49);
    groebner_row38_000000000_f(groebner_matrix, 49);
    groebner_row39_000000000_f(groebner_matrix, 49);
    let factor = groebner_matrix[(49, 163)];
    groebner_matrix[(49, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(49, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(49, 180)] -= reduction_term;
    let factor = groebner_matrix[(49, 164)];
    groebner_matrix[(49, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(49, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(49, 192)] -= reduction_term;
    let factor = -groebner_matrix[(49, 165)];
    groebner_matrix[(49, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(49, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(49, 191)] -= reduction_term;
    let factor = groebner_matrix[(49, 166)];
    groebner_matrix[(49, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(49, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(49, 183)] -= reduction_term;
    let factor = -groebner_matrix[(49, 167)];
    groebner_matrix[(49, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(49, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(49, 189)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_123(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(49, 168)];

    groebner_matrix[(49, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(49, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(49, 188)] -= reduction_term;
    let factor = groebner_matrix[(49, 169)];
    groebner_matrix[(49, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(49, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(49, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(49, 196)] -= reduction_term;
    let factor = groebner_matrix[(49, 172)];
    groebner_matrix[(49, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(49, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(49, 190)] -= reduction_term;
    let factor = -groebner_matrix[(49, 175)];
    groebner_matrix[(49, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(49, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(49, 187)] -= reduction_term;
    s_polynomial50(groebner_matrix);
    let factor = groebner_matrix[(50, 53)];
    groebner_matrix[(50, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(50, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(50, 146)] -= reduction_term;
    let factor = groebner_matrix[(50, 54)];
    groebner_matrix[(50, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(50, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(50, 117)] -= reduction_term;
    let factor = -groebner_matrix[(50, 55)];
    groebner_matrix[(50, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(50, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(50, 159)] -= reduction_term;
    let factor = groebner_matrix[(50, 56)];
    groebner_matrix[(50, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(50, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(50, 158)] -= reduction_term;
    let factor = -groebner_matrix[(50, 76)];
    groebner_matrix[(50, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(50, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(50, 142)] -= reduction_term;
    let factor = -groebner_matrix[(50, 79)];
    groebner_matrix[(50, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(50, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(50, 148)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_124(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(50, 80)];
    groebner_matrix[(50, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(50, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(50, 152)] -= reduction_term;
    let factor = groebner_matrix[(50, 82)];
    groebner_matrix[(50, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(50, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(50, 127)] -= reduction_term;
    let factor = groebner_matrix[(50, 83)];
    groebner_matrix[(50, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(50, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(50, 175)] -= reduction_term;
    let factor = groebner_matrix[(50, 85)];
    groebner_matrix[(50, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(50, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(50, 130)] -= reduction_term;
    let factor = -groebner_matrix[(50, 86)];
    groebner_matrix[(50, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(50, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(50, 172)] -= reduction_term;
    let factor = groebner_matrix[(50, 88)];
    groebner_matrix[(50, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(50, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(50, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(50, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 50);
    groebner_row41_000000000_f(groebner_matrix, 50);
    groebner_row42_000000000_f(groebner_matrix, 50);
    groebner_row43_000000000_f(groebner_matrix, 50);
    groebner_row44_000000000_f(groebner_matrix, 50);
    groebner_row45_000000000_f(groebner_matrix, 50);
    let factor = groebner_matrix[(50, 105)];
    groebner_matrix[(50, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(50, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(50, 185)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 50);
    groebner_row31_100000000_f(groebner_matrix, 50);
    groebner_row32_100000000_f(groebner_matrix, 50);
    groebner_row33_100000000_f(groebner_matrix, 50);
    groebner_row34_100000000_f(groebner_matrix, 50);
    groebner_row35_100000000_f(groebner_matrix, 50);
    groebner_row36_100000000_f(groebner_matrix, 50);
    groebner_row37_100000000_f(groebner_matrix, 50);
    groebner_row38_100000000_f(groebner_matrix, 50);
    groebner_row39_100000000_f(groebner_matrix, 50);
}

#[inline]
fn compute_basis_stage_125(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(50, 118)];

    groebner_matrix[(50, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(50, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(50, 135)] -= reduction_term;
    let factor = groebner_matrix[(50, 119)];
    groebner_matrix[(50, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(50, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(50, 183)] -= reduction_term;
    let factor = -groebner_matrix[(50, 120)];
    groebner_matrix[(50, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(50, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(50, 182)] -= reduction_term;
    let factor = groebner_matrix[(50, 121)];
    groebner_matrix[(50, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(50, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(50, 138)] -= reduction_term;
    let factor = -groebner_matrix[(50, 122)];
    groebner_matrix[(50, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(50, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(50, 180)] -= reduction_term;
    let factor = groebner_matrix[(50, 123)];
    groebner_matrix[(50, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(50, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(50, 179)] -= reduction_term;
    let factor = groebner_matrix[(50, 124)];
    groebner_matrix[(50, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(50, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(50, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(50, 195)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 50);
    groebner_row47_000000000_f(groebner_matrix, 50);
    let factor = groebner_matrix[(50, 127)];
    groebner_matrix[(50, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(50, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(50, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 50);
    groebner_row49_000000000_f(groebner_matrix, 50);
    let factor = -groebner_matrix[(50, 130)];
    groebner_matrix[(50, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(50, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(50, 178)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_126(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(50, 142)];

    groebner_matrix[(50, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(50, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(50, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(50, 196)] -= reduction_term;
    let factor = groebner_matrix[(50, 144)];
    groebner_matrix[(50, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(50, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(50, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(50, 196)] -= reduction_term;
    let factor = groebner_matrix[(50, 146)];
    groebner_matrix[(50, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(50, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(50, 185)] -= reduction_term;
    let factor = groebner_matrix[(50, 147)];
    groebner_matrix[(50, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(50, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(50, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(50, 196)] -= reduction_term;
    let factor = groebner_matrix[(50, 148)];
    groebner_matrix[(50, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(50, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(50, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 50);
    groebner_row31_000000000_f(groebner_matrix, 50);
    groebner_row32_000000000_f(groebner_matrix, 50);
    groebner_row33_000000000_f(groebner_matrix, 50);
    groebner_row34_000000000_f(groebner_matrix, 50);
    groebner_row35_000000000_f(groebner_matrix, 50);
    groebner_row36_000000000_f(groebner_matrix, 50);
    groebner_row37_000000000_f(groebner_matrix, 50);
    groebner_row38_000000000_f(groebner_matrix, 50);
    groebner_row39_000000000_f(groebner_matrix, 50);
    let factor = groebner_matrix[(50, 163)];
    groebner_matrix[(50, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(50, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(50, 180)] -= reduction_term;
    let factor = groebner_matrix[(50, 164)];
    groebner_matrix[(50, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(50, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(50, 192)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_127(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(50, 165)];

    groebner_matrix[(50, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(50, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(50, 191)] -= reduction_term;
    let factor = groebner_matrix[(50, 166)];
    groebner_matrix[(50, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(50, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(50, 183)] -= reduction_term;
    let factor = -groebner_matrix[(50, 167)];
    groebner_matrix[(50, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(50, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(50, 189)] -= reduction_term;
    let factor = groebner_matrix[(50, 168)];
    groebner_matrix[(50, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(50, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(50, 188)] -= reduction_term;
    let factor = groebner_matrix[(50, 169)];
    groebner_matrix[(50, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(50, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(50, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(50, 196)] -= reduction_term;
    let factor = groebner_matrix[(50, 172)];
    groebner_matrix[(50, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(50, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(50, 190)] -= reduction_term;
    let factor = -groebner_matrix[(50, 175)];
    groebner_matrix[(50, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(50, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(50, 187)] -= reduction_term;
    s_polynomial51(groebner_matrix);
    let factor = groebner_matrix[(51, 52)];
    groebner_matrix[(51, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(51, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(51, 162)] -= reduction_term;
    let factor = groebner_matrix[(51, 53)];
    groebner_matrix[(51, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(51, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(51, 146)] -= reduction_term;
    let factor = groebner_matrix[(51, 54)];
    groebner_matrix[(51, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(51, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(51, 117)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_128(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(51, 55)];
    groebner_matrix[(51, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(51, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(51, 159)] -= reduction_term;
    let factor = groebner_matrix[(51, 56)];
    groebner_matrix[(51, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(51, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(51, 158)] -= reduction_term;
    let factor = -groebner_matrix[(51, 76)];
    groebner_matrix[(51, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(51, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(51, 142)] -= reduction_term;
    let factor = -groebner_matrix[(51, 77)];
    groebner_matrix[(51, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(51, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(51, 143)] -= reduction_term;
    let factor = -groebner_matrix[(51, 79)];
    groebner_matrix[(51, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(51, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(51, 148)] -= reduction_term;
    let factor = -groebner_matrix[(51, 80)];
    groebner_matrix[(51, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(51, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(51, 152)] -= reduction_term;
    let factor = groebner_matrix[(51, 82)];
    groebner_matrix[(51, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(51, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(51, 127)] -= reduction_term;
    let factor = groebner_matrix[(51, 83)];
    groebner_matrix[(51, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(51, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(51, 175)] -= reduction_term;
    let factor = groebner_matrix[(51, 85)];
    groebner_matrix[(51, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(51, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(51, 130)] -= reduction_term;
    let factor = -groebner_matrix[(51, 86)];
    groebner_matrix[(51, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(51, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(51, 172)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_129(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(51, 88)];

    groebner_matrix[(51, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(51, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(51, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(51, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 51);
    groebner_row41_000000000_f(groebner_matrix, 51);
    groebner_row42_000000000_f(groebner_matrix, 51);
    groebner_row43_000000000_f(groebner_matrix, 51);
    groebner_row44_000000000_f(groebner_matrix, 51);
    groebner_row45_000000000_f(groebner_matrix, 51);
    let factor = groebner_matrix[(51, 105)];
    groebner_matrix[(51, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(51, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(51, 185)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 51);
    groebner_row31_100000000_f(groebner_matrix, 51);
    groebner_row32_100000000_f(groebner_matrix, 51);
    groebner_row33_100000000_f(groebner_matrix, 51);
    groebner_row34_100000000_f(groebner_matrix, 51);
    groebner_row35_100000000_f(groebner_matrix, 51);
    groebner_row36_100000000_f(groebner_matrix, 51);
    groebner_row37_100000000_f(groebner_matrix, 51);
    groebner_row38_100000000_f(groebner_matrix, 51);
    groebner_row39_100000000_f(groebner_matrix, 51);
    let factor = groebner_matrix[(51, 118)];
    groebner_matrix[(51, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(51, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(51, 135)] -= reduction_term;
    let factor = groebner_matrix[(51, 119)];
    groebner_matrix[(51, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(51, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(51, 183)] -= reduction_term;
    let factor = -groebner_matrix[(51, 120)];
    groebner_matrix[(51, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(51, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(51, 182)] -= reduction_term;
    let factor = groebner_matrix[(51, 121)];
    groebner_matrix[(51, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(51, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(51, 138)] -= reduction_term;
    let factor = -groebner_matrix[(51, 122)];
    groebner_matrix[(51, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(51, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(51, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_130(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(51, 123)];

    groebner_matrix[(51, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(51, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(51, 179)] -= reduction_term;
    let factor = groebner_matrix[(51, 124)];
    groebner_matrix[(51, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(51, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(51, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(51, 195)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 51);
    groebner_row47_000000000_f(groebner_matrix, 51);
    let factor = groebner_matrix[(51, 127)];
    groebner_matrix[(51, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(51, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(51, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 51);
    groebner_row49_000000000_f(groebner_matrix, 51);
    let factor = -groebner_matrix[(51, 130)];
    groebner_matrix[(51, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(51, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(51, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 51);
    let factor = groebner_matrix[(51, 142)];
    groebner_matrix[(51, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(51, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(51, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(51, 196)] -= reduction_term;
    let factor = groebner_matrix[(51, 143)];
    groebner_matrix[(51, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(51, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(51, 176)] -= reduction_term;
    let factor = groebner_matrix[(51, 144)];
    groebner_matrix[(51, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(51, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(51, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(51, 196)] -= reduction_term;
    let factor = groebner_matrix[(51, 146)];
    groebner_matrix[(51, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(51, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(51, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_131(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(51, 147)];

    groebner_matrix[(51, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(51, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(51, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(51, 196)] -= reduction_term;
    let factor = groebner_matrix[(51, 148)];
    groebner_matrix[(51, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(51, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(51, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 51);
    groebner_row31_000000000_f(groebner_matrix, 51);
    groebner_row32_000000000_f(groebner_matrix, 51);
    groebner_row33_000000000_f(groebner_matrix, 51);
    groebner_row34_000000000_f(groebner_matrix, 51);
    groebner_row35_000000000_f(groebner_matrix, 51);
    groebner_row36_000000000_f(groebner_matrix, 51);
    groebner_row37_000000000_f(groebner_matrix, 51);
    groebner_row38_000000000_f(groebner_matrix, 51);
    groebner_row39_000000000_f(groebner_matrix, 51);
    let factor = groebner_matrix[(51, 163)];
    groebner_matrix[(51, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(51, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(51, 180)] -= reduction_term;
    let factor = groebner_matrix[(51, 164)];
    groebner_matrix[(51, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(51, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(51, 192)] -= reduction_term;
    let factor = -groebner_matrix[(51, 165)];
    groebner_matrix[(51, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(51, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(51, 191)] -= reduction_term;
    let factor = groebner_matrix[(51, 166)];
    groebner_matrix[(51, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(51, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(51, 183)] -= reduction_term;
    let factor = -groebner_matrix[(51, 167)];
    groebner_matrix[(51, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(51, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(51, 189)] -= reduction_term;
    let factor = groebner_matrix[(51, 168)];
    groebner_matrix[(51, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(51, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(51, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_132(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(51, 169)];

    groebner_matrix[(51, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(51, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(51, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(51, 196)] -= reduction_term;
    let factor = groebner_matrix[(51, 172)];
    groebner_matrix[(51, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(51, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(51, 190)] -= reduction_term;
    let factor = -groebner_matrix[(51, 175)];
    groebner_matrix[(51, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(51, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(51, 187)] -= reduction_term;
    s_polynomial52(groebner_matrix);
    let factor = groebner_matrix[(52, 51)];
    groebner_matrix[(52, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(52, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(52, 114)] -= reduction_term;
    let factor = groebner_matrix[(52, 52)];
    groebner_matrix[(52, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(52, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(52, 162)] -= reduction_term;
    let factor = groebner_matrix[(52, 53)];
    groebner_matrix[(52, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(52, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(52, 146)] -= reduction_term;
    let factor = groebner_matrix[(52, 54)];
    groebner_matrix[(52, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(52, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(52, 117)] -= reduction_term;
    let factor = -groebner_matrix[(52, 55)];
    groebner_matrix[(52, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(52, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(52, 159)] -= reduction_term;
    let factor = groebner_matrix[(52, 56)];
    groebner_matrix[(52, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(52, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(52, 158)] -= reduction_term;
    groebner_row32_010000000_f(groebner_matrix, 52);
    groebner_row33_010000000_f(groebner_matrix, 52);
}

#[inline]
fn compute_basis_stage_133(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(52, 76)];

    groebner_matrix[(52, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(52, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(52, 142)] -= reduction_term;
    let factor = -groebner_matrix[(52, 77)];
    groebner_matrix[(52, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(52, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(52, 143)] -= reduction_term;
    let factor = groebner_matrix[(52, 78)];
    groebner_matrix[(52, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(52, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(52, 160)] -= reduction_term;
    let factor = -groebner_matrix[(52, 79)];
    groebner_matrix[(52, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(52, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(52, 148)] -= reduction_term;
    let factor = -groebner_matrix[(52, 80)];
    groebner_matrix[(52, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(52, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(52, 152)] -= reduction_term;
    let factor = -groebner_matrix[(52, 81)];
    groebner_matrix[(52, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(52, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(52, 157)] -= reduction_term;
    let factor = groebner_matrix[(52, 82)];
    groebner_matrix[(52, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(52, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(52, 127)] -= reduction_term;
    let factor = groebner_matrix[(52, 83)];
    groebner_matrix[(52, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(52, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(52, 175)] -= reduction_term;
    let factor = groebner_matrix[(52, 85)];
    groebner_matrix[(52, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(52, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(52, 130)] -= reduction_term;
    let factor = -groebner_matrix[(52, 86)];
    groebner_matrix[(52, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(52, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(52, 172)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_134(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(52, 88)];

    groebner_matrix[(52, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(52, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(52, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(52, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 52);
    groebner_row41_000000000_f(groebner_matrix, 52);
    groebner_row42_000000000_f(groebner_matrix, 52);
    groebner_row43_000000000_f(groebner_matrix, 52);
    groebner_row44_000000000_f(groebner_matrix, 52);
    groebner_row45_000000000_f(groebner_matrix, 52);
    let factor = groebner_matrix[(52, 105)];
    groebner_matrix[(52, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(52, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(52, 185)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 52);
    groebner_row31_100000000_f(groebner_matrix, 52);
    groebner_row32_100000000_f(groebner_matrix, 52);
    groebner_row33_100000000_f(groebner_matrix, 52);
    groebner_row34_100000000_f(groebner_matrix, 52);
    groebner_row35_100000000_f(groebner_matrix, 52);
    groebner_row36_100000000_f(groebner_matrix, 52);
    groebner_row37_100000000_f(groebner_matrix, 52);
    groebner_row38_100000000_f(groebner_matrix, 52);
    groebner_row39_100000000_f(groebner_matrix, 52);
    let factor = groebner_matrix[(52, 118)];
    groebner_matrix[(52, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(52, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(52, 135)] -= reduction_term;
    let factor = groebner_matrix[(52, 119)];
    groebner_matrix[(52, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(52, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(52, 183)] -= reduction_term;
    let factor = -groebner_matrix[(52, 120)];
    groebner_matrix[(52, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(52, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(52, 182)] -= reduction_term;
    let factor = groebner_matrix[(52, 121)];
    groebner_matrix[(52, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(52, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(52, 138)] -= reduction_term;
    let factor = -groebner_matrix[(52, 122)];
    groebner_matrix[(52, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(52, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(52, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_135(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(52, 123)];

    groebner_matrix[(52, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(52, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(52, 179)] -= reduction_term;
    let factor = groebner_matrix[(52, 124)];
    groebner_matrix[(52, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(52, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(52, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(52, 195)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 52);
    groebner_row47_000000000_f(groebner_matrix, 52);
    let factor = groebner_matrix[(52, 127)];
    groebner_matrix[(52, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(52, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(52, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 52);
    groebner_row49_000000000_f(groebner_matrix, 52);
    let factor = -groebner_matrix[(52, 130)];
    groebner_matrix[(52, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(52, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(52, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 52);
    groebner_row51_000000000_f(groebner_matrix, 52);
    let factor = groebner_matrix[(52, 142)];
    groebner_matrix[(52, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(52, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(52, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(52, 196)] -= reduction_term;
    let factor = groebner_matrix[(52, 143)];
    groebner_matrix[(52, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(52, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(52, 176)] -= reduction_term;
    let factor = groebner_matrix[(52, 144)];
    groebner_matrix[(52, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(52, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(52, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(52, 196)] -= reduction_term;
    let factor = groebner_matrix[(52, 146)];
    groebner_matrix[(52, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(52, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(52, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_136(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(52, 147)];

    groebner_matrix[(52, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(52, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(52, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(52, 196)] -= reduction_term;
    let factor = groebner_matrix[(52, 148)];
    groebner_matrix[(52, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(52, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(52, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 52);
    groebner_row31_000000000_f(groebner_matrix, 52);
    let factor = -groebner_matrix[(52, 154)];
    groebner_matrix[(52, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(52, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(52, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 52);
    groebner_row33_000000000_f(groebner_matrix, 52);
    groebner_row34_000000000_f(groebner_matrix, 52);
    groebner_row35_000000000_f(groebner_matrix, 52);
    groebner_row36_000000000_f(groebner_matrix, 52);
    groebner_row37_000000000_f(groebner_matrix, 52);
    groebner_row38_000000000_f(groebner_matrix, 52);
    groebner_row39_000000000_f(groebner_matrix, 52);
    let factor = groebner_matrix[(52, 163)];
    groebner_matrix[(52, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(52, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(52, 180)] -= reduction_term;
    let factor = groebner_matrix[(52, 164)];
    groebner_matrix[(52, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(52, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(52, 192)] -= reduction_term;
    let factor = -groebner_matrix[(52, 165)];
    groebner_matrix[(52, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(52, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(52, 191)] -= reduction_term;
    let factor = groebner_matrix[(52, 166)];
    groebner_matrix[(52, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(52, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(52, 183)] -= reduction_term;
    let factor = -groebner_matrix[(52, 167)];
    groebner_matrix[(52, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(52, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(52, 189)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_137(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(52, 168)];

    groebner_matrix[(52, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(52, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(52, 188)] -= reduction_term;
    let factor = groebner_matrix[(52, 169)];
    groebner_matrix[(52, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(52, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(52, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(52, 196)] -= reduction_term;
    let factor = groebner_matrix[(52, 172)];
    groebner_matrix[(52, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(52, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(52, 190)] -= reduction_term;
    let factor = -groebner_matrix[(52, 175)];
    groebner_matrix[(52, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(52, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(52, 187)] -= reduction_term;
    s_polynomial53(groebner_matrix);
    let factor = -groebner_matrix[(53, 50)];
    groebner_matrix[(53, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(53, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(53, 154)] -= reduction_term;
    let factor = groebner_matrix[(53, 51)];
    groebner_matrix[(53, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(53, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(53, 114)] -= reduction_term;
    let factor = groebner_matrix[(53, 52)];
    groebner_matrix[(53, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(53, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(53, 162)] -= reduction_term;
    let factor = groebner_matrix[(53, 53)];
    groebner_matrix[(53, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(53, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(53, 146)] -= reduction_term;
    let factor = groebner_matrix[(53, 54)];
    groebner_matrix[(53, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(53, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(53, 117)] -= reduction_term;
    let factor = -groebner_matrix[(53, 55)];
    groebner_matrix[(53, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(53, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(53, 159)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_138(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(53, 56)];
    groebner_matrix[(53, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(53, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(53, 158)] -= reduction_term;
    groebner_row32_010000000_f(groebner_matrix, 53);
    groebner_row33_010000000_f(groebner_matrix, 53);
    let factor = -groebner_matrix[(53, 76)];
    groebner_matrix[(53, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(53, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(53, 142)] -= reduction_term;
    let factor = -groebner_matrix[(53, 77)];
    groebner_matrix[(53, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(53, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(53, 143)] -= reduction_term;
    let factor = groebner_matrix[(53, 78)];
    groebner_matrix[(53, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(53, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(53, 160)] -= reduction_term;
    let factor = -groebner_matrix[(53, 79)];
    groebner_matrix[(53, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(53, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(53, 148)] -= reduction_term;
    let factor = -groebner_matrix[(53, 80)];
    groebner_matrix[(53, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(53, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(53, 152)] -= reduction_term;
    let factor = -groebner_matrix[(53, 81)];
    groebner_matrix[(53, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(53, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(53, 157)] -= reduction_term;
    let factor = groebner_matrix[(53, 82)];
    groebner_matrix[(53, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(53, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(53, 127)] -= reduction_term;
    let factor = groebner_matrix[(53, 83)];
    groebner_matrix[(53, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(53, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(53, 175)] -= reduction_term;
    let factor = groebner_matrix[(53, 85)];
    groebner_matrix[(53, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(53, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(53, 130)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_139(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(53, 86)];
    groebner_matrix[(53, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(53, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(53, 172)] -= reduction_term;
    let factor = groebner_matrix[(53, 88)];
    groebner_matrix[(53, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(53, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(53, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(53, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 53);
    groebner_row41_000000000_f(groebner_matrix, 53);
    groebner_row42_000000000_f(groebner_matrix, 53);
    groebner_row43_000000000_f(groebner_matrix, 53);
    groebner_row44_000000000_f(groebner_matrix, 53);
    groebner_row45_000000000_f(groebner_matrix, 53);
    let factor = groebner_matrix[(53, 105)];
    groebner_matrix[(53, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(53, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(53, 185)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 53);
    groebner_row31_100000000_f(groebner_matrix, 53);
    groebner_row32_100000000_f(groebner_matrix, 53);
    groebner_row33_100000000_f(groebner_matrix, 53);
    groebner_row34_100000000_f(groebner_matrix, 53);
    groebner_row35_100000000_f(groebner_matrix, 53);
    groebner_row36_100000000_f(groebner_matrix, 53);
    groebner_row37_100000000_f(groebner_matrix, 53);
    groebner_row38_100000000_f(groebner_matrix, 53);
    groebner_row39_100000000_f(groebner_matrix, 53);
    let factor = groebner_matrix[(53, 118)];
    groebner_matrix[(53, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(53, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(53, 135)] -= reduction_term;
    let factor = groebner_matrix[(53, 119)];
    groebner_matrix[(53, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(53, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(53, 183)] -= reduction_term;
    let factor = -groebner_matrix[(53, 120)];
    groebner_matrix[(53, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(53, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(53, 182)] -= reduction_term;
    let factor = groebner_matrix[(53, 121)];
    groebner_matrix[(53, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(53, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(53, 138)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_140(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(53, 122)];

    groebner_matrix[(53, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(53, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(53, 180)] -= reduction_term;
    let factor = groebner_matrix[(53, 123)];
    groebner_matrix[(53, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(53, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(53, 179)] -= reduction_term;
    let factor = groebner_matrix[(53, 124)];
    groebner_matrix[(53, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(53, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(53, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(53, 195)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 53);
    groebner_row47_000000000_f(groebner_matrix, 53);
    let factor = groebner_matrix[(53, 127)];
    groebner_matrix[(53, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(53, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(53, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 53);
    groebner_row49_000000000_f(groebner_matrix, 53);
    let factor = -groebner_matrix[(53, 130)];
    groebner_matrix[(53, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(53, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(53, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 53);
    groebner_row51_000000000_f(groebner_matrix, 53);
    groebner_row52_000000000_f(groebner_matrix, 53);
    let factor = groebner_matrix[(53, 142)];
    groebner_matrix[(53, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(53, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(53, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(53, 196)] -= reduction_term;
    let factor = groebner_matrix[(53, 143)];
    groebner_matrix[(53, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(53, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(53, 176)] -= reduction_term;
    let factor = groebner_matrix[(53, 144)];
    groebner_matrix[(53, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(53, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(53, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(53, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_141(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(53, 146)];

    groebner_matrix[(53, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(53, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(53, 185)] -= reduction_term;
    let factor = groebner_matrix[(53, 147)];
    groebner_matrix[(53, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(53, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(53, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(53, 196)] -= reduction_term;
    let factor = groebner_matrix[(53, 148)];
    groebner_matrix[(53, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(53, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(53, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 53);
    groebner_row31_000000000_f(groebner_matrix, 53);
    let factor = -groebner_matrix[(53, 154)];
    groebner_matrix[(53, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(53, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(53, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 53);
    groebner_row33_000000000_f(groebner_matrix, 53);
    groebner_row34_000000000_f(groebner_matrix, 53);
    groebner_row35_000000000_f(groebner_matrix, 53);
    groebner_row36_000000000_f(groebner_matrix, 53);
    groebner_row37_000000000_f(groebner_matrix, 53);
    groebner_row38_000000000_f(groebner_matrix, 53);
    groebner_row39_000000000_f(groebner_matrix, 53);
    let factor = groebner_matrix[(53, 163)];
    groebner_matrix[(53, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(53, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(53, 180)] -= reduction_term;
    let factor = groebner_matrix[(53, 164)];
    groebner_matrix[(53, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(53, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(53, 192)] -= reduction_term;
    let factor = -groebner_matrix[(53, 165)];
    groebner_matrix[(53, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(53, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(53, 191)] -= reduction_term;
    let factor = groebner_matrix[(53, 166)];
    groebner_matrix[(53, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(53, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(53, 183)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_142(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(53, 167)];

    groebner_matrix[(53, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(53, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(53, 189)] -= reduction_term;
    let factor = groebner_matrix[(53, 168)];
    groebner_matrix[(53, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(53, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(53, 188)] -= reduction_term;
    let factor = groebner_matrix[(53, 169)];
    groebner_matrix[(53, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(53, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(53, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(53, 196)] -= reduction_term;
    let factor = groebner_matrix[(53, 172)];
    groebner_matrix[(53, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(53, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(53, 190)] -= reduction_term;
    let factor = -groebner_matrix[(53, 175)];
    groebner_matrix[(53, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(53, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(53, 187)] -= reduction_term;
    s_polynomial54(groebner_matrix);
    let factor = groebner_matrix[(54, 49)];
    groebner_matrix[(54, 49)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(54, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(54, 116)] -= reduction_term;
    let factor = -groebner_matrix[(54, 50)];
    groebner_matrix[(54, 50)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(54, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(54, 154)] -= reduction_term;
    let factor = groebner_matrix[(54, 51)];
    groebner_matrix[(54, 51)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(54, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(54, 114)] -= reduction_term;
    let factor = groebner_matrix[(54, 52)];
    groebner_matrix[(54, 52)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(54, 76)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(54, 162)] -= reduction_term;
    let factor = groebner_matrix[(54, 53)];
    groebner_matrix[(54, 53)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(54, 105)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(54, 146)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_143(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(54, 54)];
    groebner_matrix[(54, 54)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(54, 80)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(54, 117)] -= reduction_term;
    let factor = -groebner_matrix[(54, 55)];
    groebner_matrix[(54, 55)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(54, 79)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(54, 159)] -= reduction_term;
    let factor = groebner_matrix[(54, 56)];
    groebner_matrix[(54, 56)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(54, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(54, 158)] -= reduction_term;
    groebner_row30_010000000_f(groebner_matrix, 54);
    groebner_row31_010000000_f(groebner_matrix, 54);
    groebner_row32_010000000_f(groebner_matrix, 54);
    groebner_row33_010000000_f(groebner_matrix, 54);
    let factor = -groebner_matrix[(54, 76)];
    groebner_matrix[(54, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(54, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(54, 142)] -= reduction_term;
    let factor = -groebner_matrix[(54, 77)];
    groebner_matrix[(54, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(54, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(54, 143)] -= reduction_term;
    let factor = groebner_matrix[(54, 78)];
    groebner_matrix[(54, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(54, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(54, 160)] -= reduction_term;
    let factor = -groebner_matrix[(54, 79)];
    groebner_matrix[(54, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(54, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(54, 148)] -= reduction_term;
    let factor = -groebner_matrix[(54, 80)];
    groebner_matrix[(54, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(54, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(54, 152)] -= reduction_term;
    let factor = -groebner_matrix[(54, 81)];
    groebner_matrix[(54, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(54, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(54, 157)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_144(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(54, 82)];

    groebner_matrix[(54, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(54, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(54, 127)] -= reduction_term;
    let factor = groebner_matrix[(54, 83)];
    groebner_matrix[(54, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(54, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(54, 175)] -= reduction_term;
    let factor = groebner_matrix[(54, 85)];
    groebner_matrix[(54, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(54, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(54, 130)] -= reduction_term;
    let factor = -groebner_matrix[(54, 86)];
    groebner_matrix[(54, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(54, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(54, 172)] -= reduction_term;
    let factor = groebner_matrix[(54, 88)];
    groebner_matrix[(54, 88)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(54, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(54, 140)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(54, 194)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 54);
    groebner_row41_000000000_f(groebner_matrix, 54);
    groebner_row42_000000000_f(groebner_matrix, 54);
    groebner_row43_000000000_f(groebner_matrix, 54);
    groebner_row44_000000000_f(groebner_matrix, 54);
    groebner_row45_000000000_f(groebner_matrix, 54);
    let factor = groebner_matrix[(54, 105)];
    groebner_matrix[(54, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(54, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(54, 185)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 54);
    groebner_row31_100000000_f(groebner_matrix, 54);
    groebner_row32_100000000_f(groebner_matrix, 54);
    groebner_row33_100000000_f(groebner_matrix, 54);
    groebner_row34_100000000_f(groebner_matrix, 54);
    groebner_row35_100000000_f(groebner_matrix, 54);
    groebner_row36_100000000_f(groebner_matrix, 54);
    groebner_row37_100000000_f(groebner_matrix, 54);
    groebner_row38_100000000_f(groebner_matrix, 54);
    groebner_row39_100000000_f(groebner_matrix, 54);
    let factor = groebner_matrix[(54, 118)];
    groebner_matrix[(54, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(54, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(54, 135)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_145(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(54, 119)];

    groebner_matrix[(54, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(54, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(54, 183)] -= reduction_term;
    let factor = -groebner_matrix[(54, 120)];
    groebner_matrix[(54, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(54, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(54, 182)] -= reduction_term;
    let factor = groebner_matrix[(54, 121)];
    groebner_matrix[(54, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(54, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(54, 138)] -= reduction_term;
    let factor = -groebner_matrix[(54, 122)];
    groebner_matrix[(54, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(54, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(54, 180)] -= reduction_term;
    let factor = groebner_matrix[(54, 123)];
    groebner_matrix[(54, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(54, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(54, 179)] -= reduction_term;
    let factor = groebner_matrix[(54, 124)];
    groebner_matrix[(54, 124)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(54, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(54, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(54, 195)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 54);
    groebner_row47_000000000_f(groebner_matrix, 54);
    let factor = groebner_matrix[(54, 127)];
    groebner_matrix[(54, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(54, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(54, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 54);
    groebner_row49_000000000_f(groebner_matrix, 54);
    let factor = -groebner_matrix[(54, 130)];
    groebner_matrix[(54, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(54, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(54, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 54);
    groebner_row51_000000000_f(groebner_matrix, 54);
    groebner_row52_000000000_f(groebner_matrix, 54);
    groebner_row53_000000000_f(groebner_matrix, 54);
}

#[inline]
fn compute_basis_stage_146(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(54, 142)];

    groebner_matrix[(54, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(54, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(54, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(54, 196)] -= reduction_term;
    let factor = groebner_matrix[(54, 143)];
    groebner_matrix[(54, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(54, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(54, 176)] -= reduction_term;
    let factor = groebner_matrix[(54, 144)];
    groebner_matrix[(54, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(54, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(54, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(54, 196)] -= reduction_term;
    let factor = groebner_matrix[(54, 146)];
    groebner_matrix[(54, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(54, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(54, 185)] -= reduction_term;
    let factor = groebner_matrix[(54, 147)];
    groebner_matrix[(54, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(54, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(54, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(54, 196)] -= reduction_term;
    let factor = groebner_matrix[(54, 148)];
    groebner_matrix[(54, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(54, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(54, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 54);
    groebner_row31_000000000_f(groebner_matrix, 54);
    let factor = -groebner_matrix[(54, 154)];
    groebner_matrix[(54, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(54, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(54, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 54);
    groebner_row33_000000000_f(groebner_matrix, 54);
    groebner_row34_000000000_f(groebner_matrix, 54);
    groebner_row35_000000000_f(groebner_matrix, 54);
    groebner_row36_000000000_f(groebner_matrix, 54);
    groebner_row37_000000000_f(groebner_matrix, 54);
    groebner_row38_000000000_f(groebner_matrix, 54);
    groebner_row39_000000000_f(groebner_matrix, 54);
}

#[inline]
fn compute_basis_stage_147(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(54, 163)];

    groebner_matrix[(54, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(54, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(54, 180)] -= reduction_term;
    let factor = groebner_matrix[(54, 164)];
    groebner_matrix[(54, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(54, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(54, 192)] -= reduction_term;
    let factor = -groebner_matrix[(54, 165)];
    groebner_matrix[(54, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(54, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(54, 191)] -= reduction_term;
    let factor = groebner_matrix[(54, 166)];
    groebner_matrix[(54, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(54, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(54, 183)] -= reduction_term;
    let factor = -groebner_matrix[(54, 167)];
    groebner_matrix[(54, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(54, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(54, 189)] -= reduction_term;
    let factor = groebner_matrix[(54, 168)];
    groebner_matrix[(54, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(54, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(54, 188)] -= reduction_term;
    let factor = groebner_matrix[(54, 169)];
    groebner_matrix[(54, 169)] = 0.0;
    let reduction_term = factor * groebner_matrix[(10, 177)];
    groebner_matrix[(54, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 186)];
    groebner_matrix[(54, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(10, 196)];
    groebner_matrix[(54, 196)] -= reduction_term;
    let factor = groebner_matrix[(54, 172)];
    groebner_matrix[(54, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(54, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(54, 190)] -= reduction_term;
    let factor = -groebner_matrix[(54, 175)];
    groebner_matrix[(54, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(54, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(54, 187)] -= reduction_term;
    s_polynomial55(groebner_matrix);
    groebner_row39_000100000_f(groebner_matrix, 55);
    groebner_row30_010000000_f(groebner_matrix, 55);
    groebner_row31_010000000_f(groebner_matrix, 55);
    groebner_row32_010000000_f(groebner_matrix, 55);
    groebner_row33_010000000_f(groebner_matrix, 55);
}

#[inline]
fn compute_basis_stage_148(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(55, 76)];

    groebner_matrix[(55, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 142)] -= reduction_term;
    let factor = -groebner_matrix[(55, 77)];
    groebner_matrix[(55, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 143)] -= reduction_term;
    let factor = groebner_matrix[(55, 78)];
    groebner_matrix[(55, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(55, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(55, 160)] -= reduction_term;
    let factor = -groebner_matrix[(55, 79)];
    groebner_matrix[(55, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 148)] -= reduction_term;
    let factor = -groebner_matrix[(55, 80)];
    groebner_matrix[(55, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 152)] -= reduction_term;
    let factor = -groebner_matrix[(55, 81)];
    groebner_matrix[(55, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 157)] -= reduction_term;
    let factor = -groebner_matrix[(55, 86)];
    groebner_matrix[(55, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(55, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(55, 172)] -= reduction_term;
    let factor = -groebner_matrix[(55, 87)];
    groebner_matrix[(55, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 163)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 55);
    groebner_row41_000000000_f(groebner_matrix, 55);
    groebner_row42_000000000_f(groebner_matrix, 55);
    groebner_row43_000000000_f(groebner_matrix, 55);
    let factor = -groebner_matrix[(55, 94)];
    groebner_matrix[(55, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 55);
    groebner_row45_000000000_f(groebner_matrix, 55);
    groebner_row30_100000000_f(groebner_matrix, 55);
    groebner_row31_100000000_f(groebner_matrix, 55);
}

#[inline]
fn compute_basis_stage_149(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(55, 109)];

    groebner_matrix[(55, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(55, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(55, 184)] -= reduction_term;
    groebner_row32_100000000_f(groebner_matrix, 55);
    groebner_row33_100000000_f(groebner_matrix, 55);
    groebner_row34_100000000_f(groebner_matrix, 55);
    groebner_row35_100000000_f(groebner_matrix, 55);
    groebner_row36_100000000_f(groebner_matrix, 55);
    groebner_row37_100000000_f(groebner_matrix, 55);
    groebner_row38_100000000_f(groebner_matrix, 55);
    groebner_row39_100000000_f(groebner_matrix, 55);
    let factor = -groebner_matrix[(55, 122)];
    groebner_matrix[(55, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(55, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(55, 180)] -= reduction_term;
    let factor = groebner_matrix[(55, 123)];
    groebner_matrix[(55, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(55, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(55, 179)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 55);
    groebner_row47_000000000_f(groebner_matrix, 55);
    let factor = groebner_matrix[(55, 127)];
    groebner_matrix[(55, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(55, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(55, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 55);
    groebner_row49_000000000_f(groebner_matrix, 55);
    let factor = -groebner_matrix[(55, 130)];
    groebner_matrix[(55, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 55);
    groebner_row51_000000000_f(groebner_matrix, 55);
    groebner_row52_000000000_f(groebner_matrix, 55);
    groebner_row53_000000000_f(groebner_matrix, 55);
    groebner_row54_000000000_f(groebner_matrix, 55);
    let factor = groebner_matrix[(55, 142)];
    groebner_matrix[(55, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(55, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(55, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(55, 196)] -= reduction_term;
    let factor = groebner_matrix[(55, 143)];
    groebner_matrix[(55, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(55, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(55, 176)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_150(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(55, 144)];

    groebner_matrix[(55, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(55, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(55, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(55, 196)] -= reduction_term;
    let factor = groebner_matrix[(55, 147)];
    groebner_matrix[(55, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(55, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(55, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(55, 196)] -= reduction_term;
    let factor = groebner_matrix[(55, 148)];
    groebner_matrix[(55, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(55, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(55, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 55);
    groebner_row31_000000000_f(groebner_matrix, 55);
    let factor = -groebner_matrix[(55, 154)];
    groebner_matrix[(55, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(55, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(55, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 55);
    groebner_row33_000000000_f(groebner_matrix, 55);
    groebner_row34_000000000_f(groebner_matrix, 55);
    groebner_row35_000000000_f(groebner_matrix, 55);
    groebner_row36_000000000_f(groebner_matrix, 55);
    groebner_row37_000000000_f(groebner_matrix, 55);
    groebner_row38_000000000_f(groebner_matrix, 55);
    groebner_row39_000000000_f(groebner_matrix, 55);
    let factor = groebner_matrix[(55, 163)];
    groebner_matrix[(55, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(55, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(55, 180)] -= reduction_term;
    let factor = -groebner_matrix[(55, 167)];
    groebner_matrix[(55, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(55, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(55, 189)] -= reduction_term;
    let factor = groebner_matrix[(55, 168)];
    groebner_matrix[(55, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(55, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(55, 188)] -= reduction_term;
    let factor = groebner_matrix[(55, 172)];
    groebner_matrix[(55, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(55, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(55, 190)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_151(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(55, 175)];

    groebner_matrix[(55, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(55, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(55, 187)] -= reduction_term;
    s_polynomial56(groebner_matrix);
    groebner_row38_000100000_f(groebner_matrix, 56);
    groebner_row39_000100000_f(groebner_matrix, 56);
    let factor = groebner_matrix[(56, 67)];
    groebner_matrix[(56, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(56, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(56, 78)] -= reduction_term;
    let factor = -groebner_matrix[(56, 68)];
    groebner_matrix[(56, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(56, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(56, 185)] -= reduction_term;
    let factor = groebner_matrix[(56, 70)];
    groebner_matrix[(56, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(56, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(56, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(56, 194)] -= reduction_term;
    groebner_row30_010000000_f(groebner_matrix, 56);
    groebner_row31_010000000_f(groebner_matrix, 56);
    groebner_row32_010000000_f(groebner_matrix, 56);
    groebner_row33_010000000_f(groebner_matrix, 56);
    let factor = -groebner_matrix[(56, 76)];
    groebner_matrix[(56, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 142)] -= reduction_term;
    let factor = -groebner_matrix[(56, 77)];
    groebner_matrix[(56, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 143)] -= reduction_term;
    let factor = groebner_matrix[(56, 78)];
    groebner_matrix[(56, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(56, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(56, 160)] -= reduction_term;
    let factor = -groebner_matrix[(56, 79)];
    groebner_matrix[(56, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 148)] -= reduction_term;
    let factor = -groebner_matrix[(56, 80)];
    groebner_matrix[(56, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 152)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_152(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(56, 81)];
    groebner_matrix[(56, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 157)] -= reduction_term;
    let factor = groebner_matrix[(56, 85)];
    groebner_matrix[(56, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(56, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(56, 130)] -= reduction_term;
    let factor = -groebner_matrix[(56, 87)];
    groebner_matrix[(56, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 163)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 56);
    groebner_row41_000000000_f(groebner_matrix, 56);
    groebner_row42_000000000_f(groebner_matrix, 56);
    groebner_row43_000000000_f(groebner_matrix, 56);
    let factor = -groebner_matrix[(56, 94)];
    groebner_matrix[(56, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 56);
    groebner_row45_000000000_f(groebner_matrix, 56);
    let factor = groebner_matrix[(56, 103)];
    groebner_matrix[(56, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(56, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(56, 114)] -= reduction_term;
    let factor = -groebner_matrix[(56, 104)];
    groebner_matrix[(56, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(56, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(56, 186)] -= reduction_term;
    let factor = groebner_matrix[(56, 105)];
    groebner_matrix[(56, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(56, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(56, 185)] -= reduction_term;
    let factor = groebner_matrix[(56, 106)];
    groebner_matrix[(56, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(56, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(56, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(56, 195)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 56);
    groebner_row31_100000000_f(groebner_matrix, 56);
    groebner_row32_100000000_f(groebner_matrix, 56);
    groebner_row33_100000000_f(groebner_matrix, 56);
    groebner_row34_100000000_f(groebner_matrix, 56);
    groebner_row35_100000000_f(groebner_matrix, 56);
}

#[inline]
fn compute_basis_stage_153(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row36_100000000_f(groebner_matrix, 56);
    groebner_row37_100000000_f(groebner_matrix, 56);
    groebner_row38_100000000_f(groebner_matrix, 56);
    groebner_row39_100000000_f(groebner_matrix, 56);
    let factor = groebner_matrix[(56, 121)];
    groebner_matrix[(56, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(56, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(56, 138)] -= reduction_term;
    let factor = -groebner_matrix[(56, 122)];
    groebner_matrix[(56, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(56, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(56, 180)] -= reduction_term;
    let factor = groebner_matrix[(56, 123)];
    groebner_matrix[(56, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(56, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(56, 179)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 56);
    groebner_row47_000000000_f(groebner_matrix, 56);
    let factor = groebner_matrix[(56, 127)];
    groebner_matrix[(56, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(56, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(56, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 56);
    groebner_row49_000000000_f(groebner_matrix, 56);
    let factor = -groebner_matrix[(56, 130)];
    groebner_matrix[(56, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 56);
    groebner_row51_000000000_f(groebner_matrix, 56);
    groebner_row52_000000000_f(groebner_matrix, 56);
    groebner_row53_000000000_f(groebner_matrix, 56);
    groebner_row54_000000000_f(groebner_matrix, 56);
    groebner_row55_000000000_f(groebner_matrix, 56);
    let factor = groebner_matrix[(56, 142)];
    groebner_matrix[(56, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(56, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(56, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(56, 196)] -= reduction_term;
    let factor = groebner_matrix[(56, 143)];
    groebner_matrix[(56, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(56, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(56, 176)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_154(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(56, 144)];

    groebner_matrix[(56, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(56, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(56, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(56, 196)] -= reduction_term;
    let factor = groebner_matrix[(56, 147)];
    groebner_matrix[(56, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(56, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(56, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(56, 196)] -= reduction_term;
    let factor = groebner_matrix[(56, 148)];
    groebner_matrix[(56, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(56, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(56, 159)] -= reduction_term;
    let factor = -groebner_matrix[(56, 149)];
    groebner_matrix[(56, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(56, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(56, 195)] -= reduction_term;
    let factor = groebner_matrix[(56, 150)];
    groebner_matrix[(56, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(56, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(56, 194)] -= reduction_term;
    let factor = groebner_matrix[(56, 151)];
    groebner_matrix[(56, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(56, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(56, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(56, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 56);
    groebner_row31_000000000_f(groebner_matrix, 56);
    groebner_row32_000000000_f(groebner_matrix, 56);
    groebner_row33_000000000_f(groebner_matrix, 56);
    groebner_row34_000000000_f(groebner_matrix, 56);
    groebner_row35_000000000_f(groebner_matrix, 56);
    groebner_row36_000000000_f(groebner_matrix, 56);
    groebner_row37_000000000_f(groebner_matrix, 56);
    groebner_row38_000000000_f(groebner_matrix, 56);
    groebner_row39_000000000_f(groebner_matrix, 56);
    let factor = groebner_matrix[(56, 163)];
    groebner_matrix[(56, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(56, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(56, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_155(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(56, 166)];

    groebner_matrix[(56, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(56, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(56, 183)] -= reduction_term;
    let factor = groebner_matrix[(56, 168)];
    groebner_matrix[(56, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(56, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(56, 188)] -= reduction_term;
    let factor = groebner_matrix[(56, 172)];
    groebner_matrix[(56, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(56, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(56, 190)] -= reduction_term;
    let factor = -groebner_matrix[(56, 175)];
    groebner_matrix[(56, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(56, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(56, 187)] -= reduction_term;
    s_polynomial57(groebner_matrix);
    groebner_row37_000100000_f(groebner_matrix, 57);
    groebner_row38_000100000_f(groebner_matrix, 57);
    groebner_row39_000100000_f(groebner_matrix, 57);
    let factor = groebner_matrix[(57, 64)];
    groebner_matrix[(57, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(57, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(57, 148)] -= reduction_term;
    let factor = groebner_matrix[(57, 65)];
    groebner_matrix[(57, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(57, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(57, 149)] -= reduction_term;
    let factor = groebner_matrix[(57, 69)];
    groebner_matrix[(57, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(57, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(57, 151)] -= reduction_term;
    let factor = -groebner_matrix[(57, 73)];
    groebner_matrix[(57, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(57, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(57, 176)] -= reduction_term;
    let factor = -groebner_matrix[(57, 76)];
    groebner_matrix[(57, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(57, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(57, 142)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_156(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(57, 77)];

    groebner_matrix[(57, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(57, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(57, 143)] -= reduction_term;
    let factor = -groebner_matrix[(57, 79)];
    groebner_matrix[(57, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(57, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(57, 148)] -= reduction_term;
    let factor = -groebner_matrix[(57, 80)];
    groebner_matrix[(57, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(57, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(57, 152)] -= reduction_term;
    let factor = groebner_matrix[(57, 84)];
    groebner_matrix[(57, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(57, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(57, 166)] -= reduction_term;
    let factor = -groebner_matrix[(57, 87)];
    groebner_matrix[(57, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(57, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(57, 163)] -= reduction_term;
    let factor = groebner_matrix[(57, 91)];
    groebner_matrix[(57, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(57, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(57, 173)] -= reduction_term;
    let factor = -groebner_matrix[(57, 94)];
    groebner_matrix[(57, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(57, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(57, 170)] -= reduction_term;
    let factor = groebner_matrix[(57, 98)];
    groebner_matrix[(57, 98)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(57, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(57, 131)] -= reduction_term;
    let factor = groebner_matrix[(57, 99)];
    groebner_matrix[(57, 99)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(57, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(57, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(57, 195)] -= reduction_term;
    let factor = groebner_matrix[(57, 100)];
    groebner_matrix[(57, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(57, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(57, 139)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_157(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(57, 101)];

    groebner_matrix[(57, 101)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(57, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(57, 140)] -= reduction_term;
    let factor = groebner_matrix[(57, 102)];
    groebner_matrix[(57, 102)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(57, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(57, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(57, 195)] -= reduction_term;
    let factor = -groebner_matrix[(57, 104)];
    groebner_matrix[(57, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(57, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(57, 186)] -= reduction_term;
    let factor = groebner_matrix[(57, 105)];
    groebner_matrix[(57, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(57, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(57, 185)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 57);
    groebner_row31_100000000_f(groebner_matrix, 57);
    let factor = -groebner_matrix[(57, 109)];
    groebner_matrix[(57, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(57, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(57, 184)] -= reduction_term;
    groebner_row32_100000000_f(groebner_matrix, 57);
    groebner_row33_100000000_f(groebner_matrix, 57);
    groebner_row34_100000000_f(groebner_matrix, 57);
    groebner_row35_100000000_f(groebner_matrix, 57);
    groebner_row36_100000000_f(groebner_matrix, 57);
    groebner_row37_100000000_f(groebner_matrix, 57);
    groebner_row38_100000000_f(groebner_matrix, 57);
    groebner_row39_100000000_f(groebner_matrix, 57);
    let factor = groebner_matrix[(57, 119)];
    groebner_matrix[(57, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(57, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(57, 183)] -= reduction_term;
    let factor = -groebner_matrix[(57, 120)];
    groebner_matrix[(57, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(57, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(57, 182)] -= reduction_term;
    let factor = -groebner_matrix[(57, 122)];
    groebner_matrix[(57, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(57, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(57, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_158(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(57, 123)];

    groebner_matrix[(57, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(57, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(57, 179)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 57);
    groebner_row47_000000000_f(groebner_matrix, 57);
    let factor = groebner_matrix[(57, 127)];
    groebner_matrix[(57, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(57, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(57, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 57);
    groebner_row49_000000000_f(groebner_matrix, 57);
    let factor = -groebner_matrix[(57, 130)];
    groebner_matrix[(57, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(57, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(57, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 57);
    groebner_row51_000000000_f(groebner_matrix, 57);
    groebner_row52_000000000_f(groebner_matrix, 57);
    groebner_row53_000000000_f(groebner_matrix, 57);
    groebner_row54_000000000_f(groebner_matrix, 57);
    groebner_row55_000000000_f(groebner_matrix, 57);
    groebner_row56_000000000_f(groebner_matrix, 57);
    let factor = groebner_matrix[(57, 142)];
    groebner_matrix[(57, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(57, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(57, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(57, 196)] -= reduction_term;
    let factor = groebner_matrix[(57, 143)];
    groebner_matrix[(57, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(57, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(57, 176)] -= reduction_term;
    let factor = groebner_matrix[(57, 144)];
    groebner_matrix[(57, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(57, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(57, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(57, 196)] -= reduction_term;
    let factor = groebner_matrix[(57, 145)];
    groebner_matrix[(57, 145)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(57, 160)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(57, 184)] -= reduction_term;
    let factor = groebner_matrix[(57, 146)];
    groebner_matrix[(57, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(57, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(57, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_159(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(57, 147)];
    groebner_matrix[(57, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(57, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(57, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(57, 196)] -= reduction_term;
    let factor = groebner_matrix[(57, 148)];
    groebner_matrix[(57, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(57, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(57, 159)] -= reduction_term;
    let factor = -groebner_matrix[(57, 149)];
    groebner_matrix[(57, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(57, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(57, 195)] -= reduction_term;
    let factor = groebner_matrix[(57, 150)];
    groebner_matrix[(57, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(57, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(57, 194)] -= reduction_term;
    let factor = groebner_matrix[(57, 151)];
    groebner_matrix[(57, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(57, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(57, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(57, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 57);
    groebner_row31_000000000_f(groebner_matrix, 57);
    let factor = -groebner_matrix[(57, 154)];
    groebner_matrix[(57, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(57, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(57, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 57);
    groebner_row33_000000000_f(groebner_matrix, 57);
    groebner_row34_000000000_f(groebner_matrix, 57);
    groebner_row35_000000000_f(groebner_matrix, 57);
    groebner_row36_000000000_f(groebner_matrix, 57);
    groebner_row37_000000000_f(groebner_matrix, 57);
    groebner_row38_000000000_f(groebner_matrix, 57);
    groebner_row39_000000000_f(groebner_matrix, 57);
    let factor = groebner_matrix[(57, 163)];
    groebner_matrix[(57, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(57, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(57, 180)] -= reduction_term;
    let factor = -groebner_matrix[(57, 165)];
    groebner_matrix[(57, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(57, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(57, 191)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_160(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(57, 166)];
    groebner_matrix[(57, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(57, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(57, 183)] -= reduction_term;
    let factor = groebner_matrix[(57, 168)];
    groebner_matrix[(57, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(57, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(57, 188)] -= reduction_term;
    let factor = groebner_matrix[(57, 172)];
    groebner_matrix[(57, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(57, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(57, 190)] -= reduction_term;
    let factor = -groebner_matrix[(57, 175)];
    groebner_matrix[(57, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(57, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(57, 187)] -= reduction_term;
    s_polynomial58(groebner_matrix);
    groebner_row36_000100000_f(groebner_matrix, 58);
    groebner_row37_000100000_f(groebner_matrix, 58);
    groebner_row38_000100000_f(groebner_matrix, 58);
    groebner_row39_000100000_f(groebner_matrix, 58);
    let factor = groebner_matrix[(58, 62)];
    groebner_matrix[(58, 62)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(58, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(58, 95)] -= reduction_term;
    let factor = groebner_matrix[(58, 63)];
    groebner_matrix[(58, 63)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(58, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(58, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(58, 194)] -= reduction_term;
    let factor = -groebner_matrix[(58, 68)];
    groebner_matrix[(58, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(58, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(58, 185)] -= reduction_term;
    groebner_row30_010000000_f(groebner_matrix, 58);
    groebner_row31_010000000_f(groebner_matrix, 58);
    groebner_row32_010000000_f(groebner_matrix, 58);
    groebner_row33_010000000_f(groebner_matrix, 58);
    let factor = -groebner_matrix[(58, 76)];
    groebner_matrix[(58, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 142)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_161(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(58, 77)];

    groebner_matrix[(58, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 143)] -= reduction_term;
    let factor = groebner_matrix[(58, 78)];
    groebner_matrix[(58, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(58, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(58, 160)] -= reduction_term;
    let factor = -groebner_matrix[(58, 79)];
    groebner_matrix[(58, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 148)] -= reduction_term;
    let factor = -groebner_matrix[(58, 80)];
    groebner_matrix[(58, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 152)] -= reduction_term;
    let factor = -groebner_matrix[(58, 81)];
    groebner_matrix[(58, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 157)] -= reduction_term;
    let factor = groebner_matrix[(58, 83)];
    groebner_matrix[(58, 83)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(58, 89)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(58, 175)] -= reduction_term;
    let factor = -groebner_matrix[(58, 87)];
    groebner_matrix[(58, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 163)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 58);
    groebner_row41_000000000_f(groebner_matrix, 58);
    groebner_row42_000000000_f(groebner_matrix, 58);
    groebner_row43_000000000_f(groebner_matrix, 58);
    let factor = -groebner_matrix[(58, 94)];
    groebner_matrix[(58, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 58);
    groebner_row45_000000000_f(groebner_matrix, 58);
    let factor = groebner_matrix[(58, 98)];
    groebner_matrix[(58, 98)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(58, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(58, 131)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_162(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(58, 99)];

    groebner_matrix[(58, 99)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(58, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(58, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(58, 195)] -= reduction_term;
    let factor = groebner_matrix[(58, 101)];
    groebner_matrix[(58, 101)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(58, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(58, 140)] -= reduction_term;
    let factor = -groebner_matrix[(58, 104)];
    groebner_matrix[(58, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(58, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(58, 186)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 58);
    groebner_row31_100000000_f(groebner_matrix, 58);
    groebner_row32_100000000_f(groebner_matrix, 58);
    groebner_row33_100000000_f(groebner_matrix, 58);
    groebner_row34_100000000_f(groebner_matrix, 58);
    groebner_row35_100000000_f(groebner_matrix, 58);
    groebner_row36_100000000_f(groebner_matrix, 58);
    groebner_row37_100000000_f(groebner_matrix, 58);
    groebner_row38_100000000_f(groebner_matrix, 58);
    groebner_row39_100000000_f(groebner_matrix, 58);
    let factor = groebner_matrix[(58, 119)];
    groebner_matrix[(58, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(58, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(58, 183)] -= reduction_term;
    let factor = -groebner_matrix[(58, 122)];
    groebner_matrix[(58, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(58, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(58, 180)] -= reduction_term;
    let factor = groebner_matrix[(58, 123)];
    groebner_matrix[(58, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(58, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(58, 179)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 58);
    groebner_row47_000000000_f(groebner_matrix, 58);
    let factor = groebner_matrix[(58, 127)];
    groebner_matrix[(58, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(58, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(58, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 58);
    groebner_row49_000000000_f(groebner_matrix, 58);
}

#[inline]
fn compute_basis_stage_163(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(58, 130)];

    groebner_matrix[(58, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 58);
    groebner_row51_000000000_f(groebner_matrix, 58);
    groebner_row52_000000000_f(groebner_matrix, 58);
    groebner_row53_000000000_f(groebner_matrix, 58);
    groebner_row54_000000000_f(groebner_matrix, 58);
    groebner_row55_000000000_f(groebner_matrix, 58);
    groebner_row56_000000000_f(groebner_matrix, 58);
    groebner_row57_000000000_f(groebner_matrix, 58);
    let factor = groebner_matrix[(58, 142)];
    groebner_matrix[(58, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(58, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(58, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(58, 196)] -= reduction_term;
    let factor = groebner_matrix[(58, 143)];
    groebner_matrix[(58, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(58, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(58, 176)] -= reduction_term;
    let factor = groebner_matrix[(58, 144)];
    groebner_matrix[(58, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(58, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(58, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(58, 196)] -= reduction_term;
    let factor = groebner_matrix[(58, 146)];
    groebner_matrix[(58, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(58, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(58, 185)] -= reduction_term;
    let factor = groebner_matrix[(58, 147)];
    groebner_matrix[(58, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(58, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(58, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(58, 196)] -= reduction_term;
    let factor = groebner_matrix[(58, 148)];
    groebner_matrix[(58, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(58, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(58, 159)] -= reduction_term;
    let factor = -groebner_matrix[(58, 149)];
    groebner_matrix[(58, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(58, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(58, 195)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 58);
}

#[inline]
fn compute_basis_stage_164(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row31_000000000_f(groebner_matrix, 58);
    groebner_row32_000000000_f(groebner_matrix, 58);
    groebner_row33_000000000_f(groebner_matrix, 58);
    groebner_row34_000000000_f(groebner_matrix, 58);
    groebner_row35_000000000_f(groebner_matrix, 58);
    groebner_row36_000000000_f(groebner_matrix, 58);
    groebner_row37_000000000_f(groebner_matrix, 58);
    groebner_row38_000000000_f(groebner_matrix, 58);
    groebner_row39_000000000_f(groebner_matrix, 58);
    let factor = groebner_matrix[(58, 163)];
    groebner_matrix[(58, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(58, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(58, 180)] -= reduction_term;
    let factor = groebner_matrix[(58, 164)];
    groebner_matrix[(58, 164)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(58, 170)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(58, 192)] -= reduction_term;
    let factor = groebner_matrix[(58, 168)];
    groebner_matrix[(58, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(58, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(58, 188)] -= reduction_term;
    let factor = groebner_matrix[(58, 172)];
    groebner_matrix[(58, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(58, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(58, 190)] -= reduction_term;
    let factor = -groebner_matrix[(58, 175)];
    groebner_matrix[(58, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(58, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(58, 187)] -= reduction_term;
    s_polynomial59(groebner_matrix);
    groebner_row35_000100000_f(groebner_matrix, 59);
    groebner_row36_000100000_f(groebner_matrix, 59);
    groebner_row37_000100000_f(groebner_matrix, 59);
    groebner_row38_000100000_f(groebner_matrix, 59);
    groebner_row39_000100000_f(groebner_matrix, 59);
    let factor = groebner_matrix[(59, 61)];
    groebner_matrix[(59, 61)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(59, 63)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(59, 66)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(59, 194)] -= reduction_term;
    let factor = groebner_matrix[(59, 62)];
    groebner_matrix[(59, 62)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(59, 74)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(59, 95)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_165(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(59, 63)];

    groebner_matrix[(59, 63)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(59, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(59, 96)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(59, 194)] -= reduction_term;
    let factor = groebner_matrix[(59, 66)];
    groebner_matrix[(59, 66)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(59, 101)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(59, 150)] -= reduction_term;
    let factor = groebner_matrix[(59, 67)];
    groebner_matrix[(59, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(59, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(59, 78)] -= reduction_term;
    groebner_row30_010000000_f(groebner_matrix, 59);
    groebner_row31_010000000_f(groebner_matrix, 59);
    groebner_row32_010000000_f(groebner_matrix, 59);
    groebner_row33_010000000_f(groebner_matrix, 59);
    let factor = -groebner_matrix[(59, 76)];
    groebner_matrix[(59, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 142)] -= reduction_term;
    let factor = -groebner_matrix[(59, 77)];
    groebner_matrix[(59, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 143)] -= reduction_term;
    let factor = groebner_matrix[(59, 78)];
    groebner_matrix[(59, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(59, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(59, 160)] -= reduction_term;
    let factor = -groebner_matrix[(59, 79)];
    groebner_matrix[(59, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 148)] -= reduction_term;
    let factor = -groebner_matrix[(59, 80)];
    groebner_matrix[(59, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 152)] -= reduction_term;
    let factor = -groebner_matrix[(59, 81)];
    groebner_matrix[(59, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 157)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_166(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(59, 82)];

    groebner_matrix[(59, 82)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(59, 90)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(59, 127)] -= reduction_term;
    let factor = -groebner_matrix[(59, 87)];
    groebner_matrix[(59, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 163)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 59);
    groebner_row41_000000000_f(groebner_matrix, 59);
    groebner_row42_000000000_f(groebner_matrix, 59);
    groebner_row43_000000000_f(groebner_matrix, 59);
    let factor = -groebner_matrix[(59, 94)];
    groebner_matrix[(59, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 59);
    groebner_row45_000000000_f(groebner_matrix, 59);
    let factor = groebner_matrix[(59, 97)];
    groebner_matrix[(59, 97)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(59, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(59, 102)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(59, 195)] -= reduction_term;
    let factor = groebner_matrix[(59, 98)];
    groebner_matrix[(59, 98)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(59, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(59, 131)] -= reduction_term;
    let factor = groebner_matrix[(59, 99)];
    groebner_matrix[(59, 99)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(59, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(59, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(59, 195)] -= reduction_term;
    let factor = groebner_matrix[(59, 100)];
    groebner_matrix[(59, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(59, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(59, 139)] -= reduction_term;
    let factor = groebner_matrix[(59, 101)];
    groebner_matrix[(59, 101)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(59, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(59, 140)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_167(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(59, 102)];

    groebner_matrix[(59, 102)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(59, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(59, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(59, 195)] -= reduction_term;
    let factor = groebner_matrix[(59, 103)];
    groebner_matrix[(59, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(59, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(59, 114)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 59);
    groebner_row31_100000000_f(groebner_matrix, 59);
    groebner_row32_100000000_f(groebner_matrix, 59);
    groebner_row33_100000000_f(groebner_matrix, 59);
    groebner_row34_100000000_f(groebner_matrix, 59);
    groebner_row35_100000000_f(groebner_matrix, 59);
    groebner_row36_100000000_f(groebner_matrix, 59);
    groebner_row37_100000000_f(groebner_matrix, 59);
    groebner_row38_100000000_f(groebner_matrix, 59);
    groebner_row39_100000000_f(groebner_matrix, 59);
    let factor = groebner_matrix[(59, 118)];
    groebner_matrix[(59, 118)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(59, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(59, 135)] -= reduction_term;
    let factor = -groebner_matrix[(59, 122)];
    groebner_matrix[(59, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(59, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(59, 180)] -= reduction_term;
    let factor = groebner_matrix[(59, 123)];
    groebner_matrix[(59, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(59, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(59, 179)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 59);
    groebner_row47_000000000_f(groebner_matrix, 59);
    let factor = groebner_matrix[(59, 127)];
    groebner_matrix[(59, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(59, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(59, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 59);
    groebner_row49_000000000_f(groebner_matrix, 59);
    let factor = -groebner_matrix[(59, 130)];
    groebner_matrix[(59, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 59);
    groebner_row51_000000000_f(groebner_matrix, 59);
    groebner_row52_000000000_f(groebner_matrix, 59);
    groebner_row53_000000000_f(groebner_matrix, 59);
    groebner_row54_000000000_f(groebner_matrix, 59);
}

#[inline]
fn compute_basis_stage_168(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row55_000000000_f(groebner_matrix, 59);
    groebner_row56_000000000_f(groebner_matrix, 59);
    groebner_row57_000000000_f(groebner_matrix, 59);
    groebner_row58_000000000_f(groebner_matrix, 59);
    let factor = groebner_matrix[(59, 142)];
    groebner_matrix[(59, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(59, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(59, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(59, 196)] -= reduction_term;
    let factor = groebner_matrix[(59, 143)];
    groebner_matrix[(59, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(59, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(59, 176)] -= reduction_term;
    let factor = groebner_matrix[(59, 144)];
    groebner_matrix[(59, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(59, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(59, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(59, 196)] -= reduction_term;
    let factor = groebner_matrix[(59, 145)];
    groebner_matrix[(59, 145)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(59, 160)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(59, 184)] -= reduction_term;
    let factor = groebner_matrix[(59, 147)];
    groebner_matrix[(59, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(59, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(59, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(59, 196)] -= reduction_term;
    let factor = groebner_matrix[(59, 148)];
    groebner_matrix[(59, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(59, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(59, 159)] -= reduction_term;
    let factor = groebner_matrix[(59, 150)];
    groebner_matrix[(59, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(59, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(59, 194)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 59);
    groebner_row31_000000000_f(groebner_matrix, 59);
    groebner_row32_000000000_f(groebner_matrix, 59);
    groebner_row33_000000000_f(groebner_matrix, 59);
    groebner_row34_000000000_f(groebner_matrix, 59);
    groebner_row35_000000000_f(groebner_matrix, 59);
    groebner_row36_000000000_f(groebner_matrix, 59);
    groebner_row37_000000000_f(groebner_matrix, 59);
    groebner_row38_000000000_f(groebner_matrix, 59);
    groebner_row39_000000000_f(groebner_matrix, 59);
}

#[inline]
fn compute_basis_stage_169(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(59, 163)];
    groebner_matrix[(59, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(59, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(59, 180)] -= reduction_term;
    let factor = groebner_matrix[(59, 168)];
    groebner_matrix[(59, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(59, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(59, 188)] -= reduction_term;
    let factor = groebner_matrix[(59, 172)];
    groebner_matrix[(59, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(59, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(59, 190)] -= reduction_term;
    let factor = -groebner_matrix[(59, 175)];
    groebner_matrix[(59, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(59, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(59, 187)] -= reduction_term;
    s_polynomial60(groebner_matrix);
    groebner_row34_000100000_f(groebner_matrix, 60);
    groebner_row35_000100000_f(groebner_matrix, 60);
    groebner_row36_000100000_f(groebner_matrix, 60);
    groebner_row37_000100000_f(groebner_matrix, 60);
    groebner_row38_000100000_f(groebner_matrix, 60);
    groebner_row39_000100000_f(groebner_matrix, 60);
    groebner_row30_010000000_f(groebner_matrix, 60);
    groebner_row31_010000000_f(groebner_matrix, 60);
    groebner_row32_010000000_f(groebner_matrix, 60);
    groebner_row33_010000000_f(groebner_matrix, 60);
    let factor = -groebner_matrix[(60, 76)];
    groebner_matrix[(60, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 142)] -= reduction_term;
    let factor = -groebner_matrix[(60, 77)];
    groebner_matrix[(60, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 143)] -= reduction_term;
    let factor = groebner_matrix[(60, 78)];
    groebner_matrix[(60, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(60, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(60, 160)] -= reduction_term;
    let factor = -groebner_matrix[(60, 79)];
    groebner_matrix[(60, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 148)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_170(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(60, 80)];

    groebner_matrix[(60, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 152)] -= reduction_term;
    let factor = -groebner_matrix[(60, 81)];
    groebner_matrix[(60, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 157)] -= reduction_term;
    let factor = -groebner_matrix[(60, 86)];
    groebner_matrix[(60, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(60, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(60, 172)] -= reduction_term;
    let factor = -groebner_matrix[(60, 87)];
    groebner_matrix[(60, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 163)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 60);
    groebner_row41_000000000_f(groebner_matrix, 60);
    groebner_row42_000000000_f(groebner_matrix, 60);
    groebner_row43_000000000_f(groebner_matrix, 60);
    let factor = -groebner_matrix[(60, 94)];
    groebner_matrix[(60, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 60);
    groebner_row45_000000000_f(groebner_matrix, 60);
    groebner_row30_100000000_f(groebner_matrix, 60);
    groebner_row31_100000000_f(groebner_matrix, 60);
    let factor = -groebner_matrix[(60, 109)];
    groebner_matrix[(60, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(60, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(60, 184)] -= reduction_term;
    groebner_row32_100000000_f(groebner_matrix, 60);
    groebner_row33_100000000_f(groebner_matrix, 60);
    groebner_row34_100000000_f(groebner_matrix, 60);
    groebner_row35_100000000_f(groebner_matrix, 60);
    groebner_row36_100000000_f(groebner_matrix, 60);
    groebner_row37_100000000_f(groebner_matrix, 60);
    groebner_row38_100000000_f(groebner_matrix, 60);
    groebner_row39_100000000_f(groebner_matrix, 60);
    let factor = -groebner_matrix[(60, 122)];
    groebner_matrix[(60, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(60, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(60, 180)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_171(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(60, 123)];

    groebner_matrix[(60, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(60, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(60, 179)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 60);
    groebner_row47_000000000_f(groebner_matrix, 60);
    let factor = groebner_matrix[(60, 127)];
    groebner_matrix[(60, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(60, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(60, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 60);
    groebner_row49_000000000_f(groebner_matrix, 60);
    let factor = -groebner_matrix[(60, 130)];
    groebner_matrix[(60, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 60);
    groebner_row51_000000000_f(groebner_matrix, 60);
    groebner_row52_000000000_f(groebner_matrix, 60);
    groebner_row53_000000000_f(groebner_matrix, 60);
    groebner_row54_000000000_f(groebner_matrix, 60);
    groebner_row55_000000000_f(groebner_matrix, 60);
    groebner_row56_000000000_f(groebner_matrix, 60);
    groebner_row57_000000000_f(groebner_matrix, 60);
    groebner_row58_000000000_f(groebner_matrix, 60);
    groebner_row59_000000000_f(groebner_matrix, 60);
    let factor = groebner_matrix[(60, 142)];
    groebner_matrix[(60, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(60, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(60, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(60, 196)] -= reduction_term;
    let factor = groebner_matrix[(60, 143)];
    groebner_matrix[(60, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(60, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(60, 176)] -= reduction_term;
    let factor = groebner_matrix[(60, 144)];
    groebner_matrix[(60, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(60, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(60, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(60, 196)] -= reduction_term;
    let factor = groebner_matrix[(60, 147)];
    groebner_matrix[(60, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(60, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(60, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(60, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_172(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(60, 148)];

    groebner_matrix[(60, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(60, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(60, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 60);
    groebner_row31_000000000_f(groebner_matrix, 60);
    let factor = -groebner_matrix[(60, 154)];
    groebner_matrix[(60, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(60, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(60, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 60);
    groebner_row33_000000000_f(groebner_matrix, 60);
    groebner_row34_000000000_f(groebner_matrix, 60);
    groebner_row35_000000000_f(groebner_matrix, 60);
    groebner_row36_000000000_f(groebner_matrix, 60);
    groebner_row37_000000000_f(groebner_matrix, 60);
    groebner_row38_000000000_f(groebner_matrix, 60);
    groebner_row39_000000000_f(groebner_matrix, 60);
    let factor = groebner_matrix[(60, 163)];
    groebner_matrix[(60, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(60, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(60, 180)] -= reduction_term;
    let factor = -groebner_matrix[(60, 167)];
    groebner_matrix[(60, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(60, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(60, 189)] -= reduction_term;
    let factor = groebner_matrix[(60, 168)];
    groebner_matrix[(60, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(60, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(60, 188)] -= reduction_term;
    let factor = groebner_matrix[(60, 172)];
    groebner_matrix[(60, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(60, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(60, 190)] -= reduction_term;
    let factor = -groebner_matrix[(60, 175)];
    groebner_matrix[(60, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(60, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(60, 187)] -= reduction_term;
    s_polynomial61(groebner_matrix);
    groebner_row33_000100000_f(groebner_matrix, 61);
    groebner_row34_000100000_f(groebner_matrix, 61);
    groebner_row35_000100000_f(groebner_matrix, 61);
    groebner_row36_000100000_f(groebner_matrix, 61);
    groebner_row37_000100000_f(groebner_matrix, 61);
    groebner_row38_000100000_f(groebner_matrix, 61);
    groebner_row39_000100000_f(groebner_matrix, 61);
    groebner_row30_010000000_f(groebner_matrix, 61);
    groebner_row31_010000000_f(groebner_matrix, 61);
    groebner_row32_010000000_f(groebner_matrix, 61);
}

#[inline]
fn compute_basis_stage_173(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row33_010000000_f(groebner_matrix, 61);
    let factor = -groebner_matrix[(61, 76)];
    groebner_matrix[(61, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 142)] -= reduction_term;
    let factor = -groebner_matrix[(61, 77)];
    groebner_matrix[(61, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 143)] -= reduction_term;
    let factor = groebner_matrix[(61, 78)];
    groebner_matrix[(61, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(61, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(61, 160)] -= reduction_term;
    let factor = -groebner_matrix[(61, 79)];
    groebner_matrix[(61, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 148)] -= reduction_term;
    let factor = -groebner_matrix[(61, 80)];
    groebner_matrix[(61, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 152)] -= reduction_term;
    let factor = -groebner_matrix[(61, 81)];
    groebner_matrix[(61, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 157)] -= reduction_term;
    let factor = -groebner_matrix[(61, 86)];
    groebner_matrix[(61, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(61, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(61, 172)] -= reduction_term;
    let factor = -groebner_matrix[(61, 87)];
    groebner_matrix[(61, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 163)] -= reduction_term;
    groebner_row40_000000000_f(groebner_matrix, 61);
    groebner_row41_000000000_f(groebner_matrix, 61);
    groebner_row42_000000000_f(groebner_matrix, 61);
    groebner_row43_000000000_f(groebner_matrix, 61);
    let factor = -groebner_matrix[(61, 94)];
    groebner_matrix[(61, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 61);
    groebner_row45_000000000_f(groebner_matrix, 61);
    groebner_row30_100000000_f(groebner_matrix, 61);
}

#[inline]
fn compute_basis_stage_174(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row31_100000000_f(groebner_matrix, 61);
    let factor = -groebner_matrix[(61, 109)];
    groebner_matrix[(61, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(61, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(61, 184)] -= reduction_term;
    groebner_row32_100000000_f(groebner_matrix, 61);
    groebner_row33_100000000_f(groebner_matrix, 61);
    groebner_row34_100000000_f(groebner_matrix, 61);
    groebner_row35_100000000_f(groebner_matrix, 61);
    groebner_row36_100000000_f(groebner_matrix, 61);
    groebner_row37_100000000_f(groebner_matrix, 61);
    groebner_row38_100000000_f(groebner_matrix, 61);
    groebner_row39_100000000_f(groebner_matrix, 61);
    let factor = -groebner_matrix[(61, 122)];
    groebner_matrix[(61, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(61, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(61, 180)] -= reduction_term;
    let factor = groebner_matrix[(61, 123)];
    groebner_matrix[(61, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(61, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(61, 179)] -= reduction_term;
    groebner_row46_000000000_f(groebner_matrix, 61);
    groebner_row47_000000000_f(groebner_matrix, 61);
    let factor = groebner_matrix[(61, 127)];
    groebner_matrix[(61, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(61, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(61, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 61);
    groebner_row49_000000000_f(groebner_matrix, 61);
    let factor = -groebner_matrix[(61, 130)];
    groebner_matrix[(61, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 61);
    groebner_row51_000000000_f(groebner_matrix, 61);
    groebner_row52_000000000_f(groebner_matrix, 61);
    groebner_row53_000000000_f(groebner_matrix, 61);
    groebner_row54_000000000_f(groebner_matrix, 61);
    groebner_row55_000000000_f(groebner_matrix, 61);
    groebner_row56_000000000_f(groebner_matrix, 61);
    groebner_row57_000000000_f(groebner_matrix, 61);
    groebner_row58_000000000_f(groebner_matrix, 61);
    groebner_row59_000000000_f(groebner_matrix, 61);
    groebner_row60_000000000_f(groebner_matrix, 61);
    let factor = groebner_matrix[(61, 142)];
    groebner_matrix[(61, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(61, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(61, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(61, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_175(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(61, 143)];
    groebner_matrix[(61, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(61, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(61, 176)] -= reduction_term;
    let factor = groebner_matrix[(61, 144)];
    groebner_matrix[(61, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(61, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(61, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(61, 196)] -= reduction_term;
    let factor = groebner_matrix[(61, 147)];
    groebner_matrix[(61, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(61, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(61, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(61, 196)] -= reduction_term;
    let factor = groebner_matrix[(61, 148)];
    groebner_matrix[(61, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(61, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(61, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 61);
    groebner_row31_000000000_f(groebner_matrix, 61);
    let factor = -groebner_matrix[(61, 154)];
    groebner_matrix[(61, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(61, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(61, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 61);
    groebner_row33_000000000_f(groebner_matrix, 61);
    groebner_row34_000000000_f(groebner_matrix, 61);
    groebner_row35_000000000_f(groebner_matrix, 61);
    groebner_row36_000000000_f(groebner_matrix, 61);
    groebner_row37_000000000_f(groebner_matrix, 61);
    groebner_row38_000000000_f(groebner_matrix, 61);
    groebner_row39_000000000_f(groebner_matrix, 61);
    let factor = groebner_matrix[(61, 163)];
    groebner_matrix[(61, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(61, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(61, 180)] -= reduction_term;
    let factor = -groebner_matrix[(61, 167)];
    groebner_matrix[(61, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(61, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(61, 189)] -= reduction_term;
    let factor = groebner_matrix[(61, 168)];
    groebner_matrix[(61, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(61, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(61, 188)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_176(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(61, 172)];
    groebner_matrix[(61, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(61, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(61, 190)] -= reduction_term;
    let factor = -groebner_matrix[(61, 175)];
    groebner_matrix[(61, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(61, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(61, 187)] -= reduction_term;
    s_polynomial62(groebner_matrix);
    groebner_row33_000100000_f(groebner_matrix, 62);
    groebner_row34_000100000_f(groebner_matrix, 62);
    groebner_row35_000100000_f(groebner_matrix, 62);
    groebner_row36_000100000_f(groebner_matrix, 62);
    groebner_row37_000100000_f(groebner_matrix, 62);
    groebner_row38_000100000_f(groebner_matrix, 62);
    groebner_row39_000100000_f(groebner_matrix, 62);
    let factor = groebner_matrix[(62, 67)];
    groebner_matrix[(62, 67)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(62, 72)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(62, 78)] -= reduction_term;
    let factor = -groebner_matrix[(62, 68)];
    groebner_matrix[(62, 68)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(62, 71)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(62, 185)] -= reduction_term;
    let factor = groebner_matrix[(62, 70)];
    groebner_matrix[(62, 70)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(62, 75)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(62, 81)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(62, 194)] -= reduction_term;
    groebner_row30_010000000_f(groebner_matrix, 62);
    groebner_row31_010000000_f(groebner_matrix, 62);
    groebner_row32_010000000_f(groebner_matrix, 62);
    groebner_row33_010000000_f(groebner_matrix, 62);
    let factor = -groebner_matrix[(62, 76)];
    groebner_matrix[(62, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 142)] -= reduction_term;
    let factor = -groebner_matrix[(62, 77)];
    groebner_matrix[(62, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 143)] -= reduction_term;
    let factor = groebner_matrix[(62, 78)];
    groebner_matrix[(62, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(62, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(62, 160)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_177(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(62, 79)];
    groebner_matrix[(62, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 148)] -= reduction_term;
    let factor = -groebner_matrix[(62, 80)];
    groebner_matrix[(62, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 152)] -= reduction_term;
    let factor = -groebner_matrix[(62, 81)];
    groebner_matrix[(62, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 157)] -= reduction_term;
    let factor = groebner_matrix[(62, 85)];
    groebner_matrix[(62, 85)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(62, 93)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(62, 130)] -= reduction_term;
    let factor = -groebner_matrix[(62, 87)];
    groebner_matrix[(62, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 163)] -= reduction_term;
    groebner_row61_010000000_f(groebner_matrix, 62);
    groebner_row41_000000000_f(groebner_matrix, 62);
    groebner_row42_000000000_f(groebner_matrix, 62);
    groebner_row43_000000000_f(groebner_matrix, 62);
    let factor = -groebner_matrix[(62, 94)];
    groebner_matrix[(62, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 62);
    groebner_row45_000000000_f(groebner_matrix, 62);
    let factor = groebner_matrix[(62, 103)];
    groebner_matrix[(62, 103)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(62, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(62, 114)] -= reduction_term;
    let factor = -groebner_matrix[(62, 104)];
    groebner_matrix[(62, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(62, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(62, 186)] -= reduction_term;
    let factor = groebner_matrix[(62, 105)];
    groebner_matrix[(62, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(62, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(62, 185)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_178(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(62, 106)];

    groebner_matrix[(62, 106)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(62, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(62, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(62, 195)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 62);
    groebner_row31_100000000_f(groebner_matrix, 62);
    groebner_row32_100000000_f(groebner_matrix, 62);
    groebner_row33_100000000_f(groebner_matrix, 62);
    groebner_row34_100000000_f(groebner_matrix, 62);
    groebner_row35_100000000_f(groebner_matrix, 62);
    groebner_row36_100000000_f(groebner_matrix, 62);
    groebner_row37_100000000_f(groebner_matrix, 62);
    groebner_row38_100000000_f(groebner_matrix, 62);
    groebner_row39_100000000_f(groebner_matrix, 62);
    let factor = groebner_matrix[(62, 121)];
    groebner_matrix[(62, 121)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(62, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(62, 138)] -= reduction_term;
    let factor = -groebner_matrix[(62, 122)];
    groebner_matrix[(62, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(62, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(62, 180)] -= reduction_term;
    let factor = groebner_matrix[(62, 123)];
    groebner_matrix[(62, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(62, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(62, 179)] -= reduction_term;
    groebner_row61_100000000_f(groebner_matrix, 62);
    groebner_row47_000000000_f(groebner_matrix, 62);
    let factor = groebner_matrix[(62, 127)];
    groebner_matrix[(62, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(62, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(62, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 62);
    groebner_row49_000000000_f(groebner_matrix, 62);
    let factor = -groebner_matrix[(62, 130)];
    groebner_matrix[(62, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 62);
    groebner_row51_000000000_f(groebner_matrix, 62);
    groebner_row52_000000000_f(groebner_matrix, 62);
    groebner_row53_000000000_f(groebner_matrix, 62);
    groebner_row54_000000000_f(groebner_matrix, 62);
    groebner_row55_000000000_f(groebner_matrix, 62);
    groebner_row56_000000000_f(groebner_matrix, 62);
    groebner_row57_000000000_f(groebner_matrix, 62);
    groebner_row58_000000000_f(groebner_matrix, 62);
    groebner_row59_000000000_f(groebner_matrix, 62);
    groebner_row60_000000000_f(groebner_matrix, 62);
}

#[inline]
fn compute_basis_stage_179(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(62, 142)];
    groebner_matrix[(62, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(62, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(62, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(62, 196)] -= reduction_term;
    let factor = groebner_matrix[(62, 143)];
    groebner_matrix[(62, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(62, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(62, 176)] -= reduction_term;
    let factor = groebner_matrix[(62, 144)];
    groebner_matrix[(62, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(62, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(62, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(62, 196)] -= reduction_term;
    let factor = groebner_matrix[(62, 147)];
    groebner_matrix[(62, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(62, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(62, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(62, 196)] -= reduction_term;
    let factor = groebner_matrix[(62, 148)];
    groebner_matrix[(62, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(62, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(62, 159)] -= reduction_term;
    let factor = -groebner_matrix[(62, 149)];
    groebner_matrix[(62, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(62, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(62, 195)] -= reduction_term;
    let factor = groebner_matrix[(62, 150)];
    groebner_matrix[(62, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(62, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(62, 194)] -= reduction_term;
    let factor = groebner_matrix[(62, 151)];
    groebner_matrix[(62, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(62, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(62, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(62, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 62);
    groebner_row31_000000000_f(groebner_matrix, 62);
    groebner_row32_000000000_f(groebner_matrix, 62);
    groebner_row33_000000000_f(groebner_matrix, 62);
    groebner_row34_000000000_f(groebner_matrix, 62);
    groebner_row35_000000000_f(groebner_matrix, 62);
}

#[inline]
fn compute_basis_stage_180(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row36_000000000_f(groebner_matrix, 62);
    groebner_row37_000000000_f(groebner_matrix, 62);
    groebner_row38_000000000_f(groebner_matrix, 62);
    groebner_row39_000000000_f(groebner_matrix, 62);
    let factor = groebner_matrix[(62, 163)];
    groebner_matrix[(62, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(62, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(62, 180)] -= reduction_term;
    let factor = groebner_matrix[(62, 166)];
    groebner_matrix[(62, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(62, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(62, 183)] -= reduction_term;
    let factor = groebner_matrix[(62, 168)];
    groebner_matrix[(62, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(62, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(62, 188)] -= reduction_term;
    groebner_row61_000000000_f(groebner_matrix, 62);
    let factor = groebner_matrix[(62, 172)];
    groebner_matrix[(62, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(62, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(62, 190)] -= reduction_term;
    let factor = -groebner_matrix[(62, 175)];
    groebner_matrix[(62, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(62, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(62, 187)] -= reduction_term;
    s_polynomial63(groebner_matrix);
    groebner_row32_000100000_f(groebner_matrix, 63);
    groebner_row33_000100000_f(groebner_matrix, 63);
    groebner_row34_000100000_f(groebner_matrix, 63);
    groebner_row35_000100000_f(groebner_matrix, 63);
    groebner_row36_000100000_f(groebner_matrix, 63);
    groebner_row37_000100000_f(groebner_matrix, 63);
    groebner_row38_000100000_f(groebner_matrix, 63);
    groebner_row39_000100000_f(groebner_matrix, 63);
    groebner_row30_010000000_f(groebner_matrix, 63);
    groebner_row31_010000000_f(groebner_matrix, 63);
    groebner_row32_010000000_f(groebner_matrix, 63);
    groebner_row33_010000000_f(groebner_matrix, 63);
    let factor = -groebner_matrix[(63, 76)];
    groebner_matrix[(63, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 142)] -= reduction_term;
    let factor = -groebner_matrix[(63, 77)];
    groebner_matrix[(63, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 143)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_181(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(63, 78)];

    groebner_matrix[(63, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(63, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(63, 160)] -= reduction_term;
    let factor = -groebner_matrix[(63, 79)];
    groebner_matrix[(63, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 148)] -= reduction_term;
    let factor = -groebner_matrix[(63, 80)];
    groebner_matrix[(63, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 152)] -= reduction_term;
    let factor = -groebner_matrix[(63, 81)];
    groebner_matrix[(63, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 157)] -= reduction_term;
    let factor = -groebner_matrix[(63, 86)];
    groebner_matrix[(63, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(63, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(63, 172)] -= reduction_term;
    let factor = -groebner_matrix[(63, 87)];
    groebner_matrix[(63, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 163)] -= reduction_term;
    groebner_row61_010000000_f(groebner_matrix, 63);
    groebner_row62_010000000_f(groebner_matrix, 63);
    groebner_row42_000000000_f(groebner_matrix, 63);
    groebner_row43_000000000_f(groebner_matrix, 63);
    let factor = -groebner_matrix[(63, 94)];
    groebner_matrix[(63, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 63);
    groebner_row45_000000000_f(groebner_matrix, 63);
    groebner_row30_100000000_f(groebner_matrix, 63);
    groebner_row31_100000000_f(groebner_matrix, 63);
    let factor = -groebner_matrix[(63, 109)];
    groebner_matrix[(63, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(63, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(63, 184)] -= reduction_term;
    groebner_row32_100000000_f(groebner_matrix, 63);
    groebner_row33_100000000_f(groebner_matrix, 63);
    groebner_row34_100000000_f(groebner_matrix, 63);
    groebner_row35_100000000_f(groebner_matrix, 63);
    groebner_row36_100000000_f(groebner_matrix, 63);
    groebner_row37_100000000_f(groebner_matrix, 63);
    groebner_row38_100000000_f(groebner_matrix, 63);
}

#[inline]
fn compute_basis_stage_182(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row39_100000000_f(groebner_matrix, 63);
    let factor = -groebner_matrix[(63, 122)];
    groebner_matrix[(63, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(63, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(63, 180)] -= reduction_term;
    let factor = groebner_matrix[(63, 123)];
    groebner_matrix[(63, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(63, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(63, 179)] -= reduction_term;
    groebner_row61_100000000_f(groebner_matrix, 63);
    groebner_row62_100000000_f(groebner_matrix, 63);
    let factor = groebner_matrix[(63, 127)];
    groebner_matrix[(63, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(63, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(63, 181)] -= reduction_term;
    groebner_row48_000000000_f(groebner_matrix, 63);
    groebner_row49_000000000_f(groebner_matrix, 63);
    let factor = -groebner_matrix[(63, 130)];
    groebner_matrix[(63, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 63);
    groebner_row51_000000000_f(groebner_matrix, 63);
    groebner_row52_000000000_f(groebner_matrix, 63);
    groebner_row53_000000000_f(groebner_matrix, 63);
    groebner_row54_000000000_f(groebner_matrix, 63);
    groebner_row55_000000000_f(groebner_matrix, 63);
    groebner_row56_000000000_f(groebner_matrix, 63);
    groebner_row57_000000000_f(groebner_matrix, 63);
    groebner_row58_000000000_f(groebner_matrix, 63);
    groebner_row59_000000000_f(groebner_matrix, 63);
    groebner_row60_000000000_f(groebner_matrix, 63);
    let factor = groebner_matrix[(63, 142)];
    groebner_matrix[(63, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(63, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(63, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(63, 196)] -= reduction_term;
    let factor = groebner_matrix[(63, 143)];
    groebner_matrix[(63, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(63, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(63, 176)] -= reduction_term;
    let factor = groebner_matrix[(63, 144)];
    groebner_matrix[(63, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(63, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(63, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(63, 196)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_183(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(63, 147)];
    groebner_matrix[(63, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(63, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(63, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(63, 196)] -= reduction_term;
    let factor = groebner_matrix[(63, 148)];
    groebner_matrix[(63, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(63, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(63, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 63);
    groebner_row31_000000000_f(groebner_matrix, 63);
    let factor = -groebner_matrix[(63, 154)];
    groebner_matrix[(63, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(63, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(63, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 63);
    groebner_row33_000000000_f(groebner_matrix, 63);
    groebner_row34_000000000_f(groebner_matrix, 63);
    groebner_row35_000000000_f(groebner_matrix, 63);
    groebner_row36_000000000_f(groebner_matrix, 63);
    groebner_row37_000000000_f(groebner_matrix, 63);
    groebner_row38_000000000_f(groebner_matrix, 63);
    groebner_row39_000000000_f(groebner_matrix, 63);
    let factor = groebner_matrix[(63, 163)];
    groebner_matrix[(63, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(63, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(63, 180)] -= reduction_term;
    let factor = -groebner_matrix[(63, 167)];
    groebner_matrix[(63, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(63, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(63, 189)] -= reduction_term;
    let factor = groebner_matrix[(63, 168)];
    groebner_matrix[(63, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(63, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(63, 188)] -= reduction_term;
    groebner_row61_000000000_f(groebner_matrix, 63);
    groebner_row62_000000000_f(groebner_matrix, 63);
    let factor = groebner_matrix[(63, 172)];
    groebner_matrix[(63, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(63, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(63, 190)] -= reduction_term;
    let factor = -groebner_matrix[(63, 175)];
    groebner_matrix[(63, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(63, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(63, 187)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_184(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    s_polynomial64(groebner_matrix);
    groebner_row35_000100000_f(groebner_matrix, 64);
    groebner_row36_000100000_f(groebner_matrix, 64);
    groebner_row37_000100000_f(groebner_matrix, 64);
    groebner_row38_000100000_f(groebner_matrix, 64);
    groebner_row39_000100000_f(groebner_matrix, 64);
    let factor = groebner_matrix[(64, 64)];
    groebner_matrix[(64, 64)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(64, 98)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(64, 148)] -= reduction_term;
    let factor = groebner_matrix[(64, 65)];
    groebner_matrix[(64, 65)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(64, 99)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(64, 149)] -= reduction_term;
    let factor = groebner_matrix[(64, 69)];
    groebner_matrix[(64, 69)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(64, 104)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(64, 151)] -= reduction_term;
    let factor = -groebner_matrix[(64, 73)];
    groebner_matrix[(64, 73)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(64, 77)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(64, 176)] -= reduction_term;
    let factor = -groebner_matrix[(64, 76)];
    groebner_matrix[(64, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(64, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(64, 142)] -= reduction_term;
    let factor = -groebner_matrix[(64, 77)];
    groebner_matrix[(64, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(64, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(64, 143)] -= reduction_term;
    let factor = -groebner_matrix[(64, 79)];
    groebner_matrix[(64, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(64, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(64, 148)] -= reduction_term;
    let factor = -groebner_matrix[(64, 80)];
    groebner_matrix[(64, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(64, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(64, 152)] -= reduction_term;
    let factor = groebner_matrix[(64, 84)];
    groebner_matrix[(64, 84)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(64, 119)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(64, 166)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_185(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(64, 87)];

    groebner_matrix[(64, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(64, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(64, 163)] -= reduction_term;
    let factor = groebner_matrix[(64, 91)];
    groebner_matrix[(64, 91)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(64, 126)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(64, 173)] -= reduction_term;
    let factor = -groebner_matrix[(64, 94)];
    groebner_matrix[(64, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(64, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(64, 170)] -= reduction_term;
    let factor = groebner_matrix[(64, 98)];
    groebner_matrix[(64, 98)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(64, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(64, 131)] -= reduction_term;
    let factor = groebner_matrix[(64, 99)];
    groebner_matrix[(64, 99)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(64, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(64, 132)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(64, 195)] -= reduction_term;
    let factor = groebner_matrix[(64, 100)];
    groebner_matrix[(64, 100)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(64, 115)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(64, 139)] -= reduction_term;
    let factor = groebner_matrix[(64, 101)];
    groebner_matrix[(64, 101)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(64, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(64, 140)] -= reduction_term;
    let factor = groebner_matrix[(64, 102)];
    groebner_matrix[(64, 102)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(64, 117)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(64, 141)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(64, 195)] -= reduction_term;
    let factor = -groebner_matrix[(64, 104)];
    groebner_matrix[(64, 104)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(64, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(64, 186)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_186(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(64, 105)];

    groebner_matrix[(64, 105)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(64, 112)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(64, 185)] -= reduction_term;
    groebner_row30_100000000_f(groebner_matrix, 64);
    groebner_row31_100000000_f(groebner_matrix, 64);
    let factor = -groebner_matrix[(64, 109)];
    groebner_matrix[(64, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(64, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(64, 184)] -= reduction_term;
    groebner_row32_100000000_f(groebner_matrix, 64);
    groebner_row33_100000000_f(groebner_matrix, 64);
    groebner_row34_100000000_f(groebner_matrix, 64);
    groebner_row35_100000000_f(groebner_matrix, 64);
    groebner_row36_100000000_f(groebner_matrix, 64);
    groebner_row37_100000000_f(groebner_matrix, 64);
    groebner_row38_100000000_f(groebner_matrix, 64);
    groebner_row39_100000000_f(groebner_matrix, 64);
    let factor = groebner_matrix[(64, 119)];
    groebner_matrix[(64, 119)] = 0.0;
    let reduction_term = factor * groebner_matrix[(20, 170)];
    groebner_matrix[(64, 125)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(20, 192)];
    groebner_matrix[(64, 183)] -= reduction_term;
    let factor = -groebner_matrix[(64, 120)];
    groebner_matrix[(64, 120)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(64, 133)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(64, 182)] -= reduction_term;
    let factor = -groebner_matrix[(64, 122)];
    groebner_matrix[(64, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(64, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(64, 180)] -= reduction_term;
    let factor = groebner_matrix[(64, 123)];
    groebner_matrix[(64, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(64, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(64, 179)] -= reduction_term;
    groebner_row61_100000000_f(groebner_matrix, 64);
    groebner_row62_100000000_f(groebner_matrix, 64);
    let factor = groebner_matrix[(64, 127)];
    groebner_matrix[(64, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(64, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(64, 181)] -= reduction_term;
    groebner_row63_100000000_f(groebner_matrix, 64);
    groebner_row49_000000000_f(groebner_matrix, 64);
    let factor = -groebner_matrix[(64, 130)];
    groebner_matrix[(64, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(64, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(64, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 64);
}

#[inline]
fn compute_basis_stage_187(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    groebner_row51_000000000_f(groebner_matrix, 64);
    groebner_row52_000000000_f(groebner_matrix, 64);
    groebner_row53_000000000_f(groebner_matrix, 64);
    groebner_row54_000000000_f(groebner_matrix, 64);
    groebner_row55_000000000_f(groebner_matrix, 64);
    groebner_row56_000000000_f(groebner_matrix, 64);
    groebner_row57_000000000_f(groebner_matrix, 64);
    groebner_row58_000000000_f(groebner_matrix, 64);
    groebner_row59_000000000_f(groebner_matrix, 64);
    groebner_row60_000000000_f(groebner_matrix, 64);
    let factor = groebner_matrix[(64, 142)];
    groebner_matrix[(64, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(64, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(64, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(64, 196)] -= reduction_term;
    let factor = groebner_matrix[(64, 143)];
    groebner_matrix[(64, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(64, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(64, 176)] -= reduction_term;
    let factor = groebner_matrix[(64, 144)];
    groebner_matrix[(64, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(64, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(64, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(64, 196)] -= reduction_term;
    let factor = groebner_matrix[(64, 145)];
    groebner_matrix[(64, 145)] = 0.0;
    let reduction_term = factor * groebner_matrix[(24, 160)];
    groebner_matrix[(64, 160)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(24, 184)];
    groebner_matrix[(64, 184)] -= reduction_term;
    let factor = groebner_matrix[(64, 146)];
    groebner_matrix[(64, 146)] = 0.0;
    let reduction_term = factor * groebner_matrix[(23, 161)];
    groebner_matrix[(64, 161)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(23, 185)];
    groebner_matrix[(64, 185)] -= reduction_term;
    let factor = groebner_matrix[(64, 147)];
    groebner_matrix[(64, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(64, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(64, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(64, 196)] -= reduction_term;
    let factor = groebner_matrix[(64, 148)];
    groebner_matrix[(64, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(64, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(64, 159)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_188(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(64, 149)];

    groebner_matrix[(64, 149)] = 0.0;
    let reduction_term = factor * groebner_matrix[(19, 152)];
    groebner_matrix[(64, 152)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(19, 195)];
    groebner_matrix[(64, 195)] -= reduction_term;
    let factor = groebner_matrix[(64, 150)];
    groebner_matrix[(64, 150)] = 0.0;
    let reduction_term = factor * groebner_matrix[(28, 157)];
    groebner_matrix[(64, 157)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(28, 194)];
    groebner_matrix[(64, 194)] -= reduction_term;
    let factor = groebner_matrix[(64, 151)];
    groebner_matrix[(64, 151)] = 0.0;
    let reduction_term = factor * groebner_matrix[(13, 156)];
    groebner_matrix[(64, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 162)];
    groebner_matrix[(64, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(13, 196)];
    groebner_matrix[(64, 196)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 64);
    groebner_row31_000000000_f(groebner_matrix, 64);
    let factor = -groebner_matrix[(64, 154)];
    groebner_matrix[(64, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(64, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(64, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 64);
    groebner_row33_000000000_f(groebner_matrix, 64);
    groebner_row34_000000000_f(groebner_matrix, 64);
    groebner_row35_000000000_f(groebner_matrix, 64);
    groebner_row36_000000000_f(groebner_matrix, 64);
    groebner_row37_000000000_f(groebner_matrix, 64);
    groebner_row38_000000000_f(groebner_matrix, 64);
    groebner_row39_000000000_f(groebner_matrix, 64);
    let factor = groebner_matrix[(64, 163)];
    groebner_matrix[(64, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(64, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(64, 180)] -= reduction_term;
    let factor = -groebner_matrix[(64, 165)];
    groebner_matrix[(64, 165)] = 0.0;
    let reduction_term = factor * groebner_matrix[(29, 178)];
    groebner_matrix[(64, 178)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(29, 191)];
    groebner_matrix[(64, 191)] -= reduction_term;
    let factor = groebner_matrix[(64, 166)];
    groebner_matrix[(64, 166)] = 0.0;
    let reduction_term = factor * groebner_matrix[(11, 174)];
    groebner_matrix[(64, 174)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(11, 183)];
    groebner_matrix[(64, 183)] -= reduction_term;
    let factor = groebner_matrix[(64, 168)];
    groebner_matrix[(64, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(64, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(64, 188)] -= reduction_term;
    groebner_row61_000000000_f(groebner_matrix, 64);
    groebner_row62_000000000_f(groebner_matrix, 64);
}

#[inline]
fn compute_basis_stage_189(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(64, 172)];

    groebner_matrix[(64, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(64, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(64, 190)] -= reduction_term;
    groebner_row63_000000000_f(groebner_matrix, 64);
    let factor = -groebner_matrix[(64, 175)];
    groebner_matrix[(64, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(64, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(64, 187)] -= reduction_term;
    s_polynomial65(groebner_matrix);
    let factor = -groebner_matrix[(65, 30)];
    groebner_matrix[(65, 30)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(65, 34)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(65, 168)] -= reduction_term;
    groebner_row32_000100000_f(groebner_matrix, 65);
    groebner_row33_000100000_f(groebner_matrix, 65);
    groebner_row34_000100000_f(groebner_matrix, 65);
    groebner_row35_000100000_f(groebner_matrix, 65);
    groebner_row36_000100000_f(groebner_matrix, 65);
    groebner_row37_000100000_f(groebner_matrix, 65);
    groebner_row38_000100000_f(groebner_matrix, 65);
    groebner_row39_000100000_f(groebner_matrix, 65);
    groebner_row30_010000000_f(groebner_matrix, 65);
    groebner_row31_010000000_f(groebner_matrix, 65);
    groebner_row32_010000000_f(groebner_matrix, 65);
    groebner_row33_010000000_f(groebner_matrix, 65);
    let factor = -groebner_matrix[(65, 76)];
    groebner_matrix[(65, 76)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 107)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 142)] -= reduction_term;
    let factor = -groebner_matrix[(65, 77)];
    groebner_matrix[(65, 77)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 108)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 143)] -= reduction_term;
    let factor = groebner_matrix[(65, 78)];
    groebner_matrix[(65, 78)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(65, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(65, 160)] -= reduction_term;
    let factor = -groebner_matrix[(65, 79)];
    groebner_matrix[(65, 79)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 110)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 148)] -= reduction_term;
    let factor = -groebner_matrix[(65, 80)];
    groebner_matrix[(65, 80)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 111)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 152)] -= reduction_term;
}

#[inline]
fn compute_basis_stage_190(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(65, 81)];

    groebner_matrix[(65, 81)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 116)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 157)] -= reduction_term;
    let factor = -groebner_matrix[(65, 86)];
    groebner_matrix[(65, 86)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(65, 92)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(65, 172)] -= reduction_term;
    let factor = -groebner_matrix[(65, 87)];
    groebner_matrix[(65, 87)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 122)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 163)] -= reduction_term;
    groebner_row61_010000000_f(groebner_matrix, 65);
    groebner_row62_010000000_f(groebner_matrix, 65);
    groebner_row63_010000000_f(groebner_matrix, 65);
    groebner_row64_010000000_f(groebner_matrix, 65);
    let factor = -groebner_matrix[(65, 94)];
    groebner_matrix[(65, 94)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 129)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 170)] -= reduction_term;
    groebner_row44_000000000_f(groebner_matrix, 65);
    groebner_row45_000000000_f(groebner_matrix, 65);
    groebner_row30_100000000_f(groebner_matrix, 65);
    groebner_row31_100000000_f(groebner_matrix, 65);
    let factor = -groebner_matrix[(65, 109)];
    groebner_matrix[(65, 109)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(65, 113)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(65, 184)] -= reduction_term;
    groebner_row32_100000000_f(groebner_matrix, 65);
    groebner_row33_100000000_f(groebner_matrix, 65);
    groebner_row34_100000000_f(groebner_matrix, 65);
    groebner_row35_100000000_f(groebner_matrix, 65);
    groebner_row36_100000000_f(groebner_matrix, 65);
    groebner_row37_100000000_f(groebner_matrix, 65);
    groebner_row38_100000000_f(groebner_matrix, 65);
    groebner_row39_100000000_f(groebner_matrix, 65);
    let factor = -groebner_matrix[(65, 122)];
    groebner_matrix[(65, 122)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(65, 128)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(65, 180)] -= reduction_term;
    let factor = groebner_matrix[(65, 123)];
    groebner_matrix[(65, 123)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(65, 136)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(65, 179)] -= reduction_term;
    groebner_row61_100000000_f(groebner_matrix, 65);
    groebner_row62_100000000_f(groebner_matrix, 65);
}

#[inline]
fn compute_basis_stage_191(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(65, 127)];

    groebner_matrix[(65, 127)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(65, 134)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(65, 181)] -= reduction_term;
    groebner_row63_100000000_f(groebner_matrix, 65);
    groebner_row64_100000000_f(groebner_matrix, 65);
    let factor = -groebner_matrix[(65, 130)];
    groebner_matrix[(65, 130)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 137)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 178)] -= reduction_term;
    groebner_row50_000000000_f(groebner_matrix, 65);
    groebner_row51_000000000_f(groebner_matrix, 65);
    groebner_row52_000000000_f(groebner_matrix, 65);
    groebner_row53_000000000_f(groebner_matrix, 65);
    groebner_row54_000000000_f(groebner_matrix, 65);
    groebner_row55_000000000_f(groebner_matrix, 65);
    groebner_row56_000000000_f(groebner_matrix, 65);
    groebner_row57_000000000_f(groebner_matrix, 65);
    groebner_row58_000000000_f(groebner_matrix, 65);
    groebner_row59_000000000_f(groebner_matrix, 65);
    groebner_row60_000000000_f(groebner_matrix, 65);
    let factor = groebner_matrix[(65, 142)];
    groebner_matrix[(65, 142)] = 0.0;
    let reduction_term = factor * groebner_matrix[(15, 144)];
    groebner_matrix[(65, 144)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 147)];
    groebner_matrix[(65, 147)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(15, 196)];
    groebner_matrix[(65, 196)] -= reduction_term;
    let factor = groebner_matrix[(65, 143)];
    groebner_matrix[(65, 143)] = 0.0;
    let reduction_term = factor * groebner_matrix[(26, 155)];
    groebner_matrix[(65, 155)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(26, 176)];
    groebner_matrix[(65, 176)] -= reduction_term;
    let factor = groebner_matrix[(65, 144)];
    groebner_matrix[(65, 144)] = 0.0;
    let reduction_term = factor * groebner_matrix[(25, 156)];
    groebner_matrix[(65, 156)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 177)];
    groebner_matrix[(65, 177)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(25, 196)];
    groebner_matrix[(65, 196)] -= reduction_term;
    let factor = groebner_matrix[(65, 147)];
    groebner_matrix[(65, 147)] = 0.0;
    let reduction_term = factor * groebner_matrix[(22, 162)];
    groebner_matrix[(65, 162)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 186)];
    groebner_matrix[(65, 186)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(22, 196)];
    groebner_matrix[(65, 196)] -= reduction_term;
    let factor = groebner_matrix[(65, 148)];
    groebner_matrix[(65, 148)] = 0.0;
    let reduction_term = factor * groebner_matrix[(14, 153)];
    groebner_matrix[(65, 153)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(14, 159)];
    groebner_matrix[(65, 159)] -= reduction_term;
    groebner_row30_000000000_f(groebner_matrix, 65);
    groebner_row31_000000000_f(groebner_matrix, 65);
}

#[inline]
fn compute_basis_stage_192(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = -groebner_matrix[(65, 154)];
    groebner_matrix[(65, 154)] = 0.0;
    let reduction_term = factor * groebner_matrix[(16, 158)];
    groebner_matrix[(65, 158)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(16, 193)];
    groebner_matrix[(65, 193)] -= reduction_term;
    groebner_row32_000000000_f(groebner_matrix, 65);
    groebner_row33_000000000_f(groebner_matrix, 65);
    groebner_row34_000000000_f(groebner_matrix, 65);
    groebner_row35_000000000_f(groebner_matrix, 65);
    groebner_row36_000000000_f(groebner_matrix, 65);
    groebner_row37_000000000_f(groebner_matrix, 65);
    groebner_row38_000000000_f(groebner_matrix, 65);
    groebner_row39_000000000_f(groebner_matrix, 65);
    let factor = groebner_matrix[(65, 163)];
    groebner_matrix[(65, 163)] = 0.0;
    let reduction_term = factor * groebner_matrix[(12, 171)];
    groebner_matrix[(65, 171)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(12, 180)];
    groebner_matrix[(65, 180)] -= reduction_term;
    let factor = -groebner_matrix[(65, 167)];
    groebner_matrix[(65, 167)] = 0.0;
    let reduction_term = factor * groebner_matrix[(21, 173)];
    groebner_matrix[(65, 173)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(21, 189)];
    groebner_matrix[(65, 189)] -= reduction_term;
    let factor = groebner_matrix[(65, 168)];
    groebner_matrix[(65, 168)] = 0.0;
    let reduction_term = factor * groebner_matrix[(27, 181)];
    groebner_matrix[(65, 181)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(27, 188)];
    groebner_matrix[(65, 188)] -= reduction_term;
    groebner_row61_000000000_f(groebner_matrix, 65);
    groebner_row62_000000000_f(groebner_matrix, 65);
    let factor = groebner_matrix[(65, 172)];
    groebner_matrix[(65, 172)] = 0.0;
    let reduction_term = factor * groebner_matrix[(17, 179)];
    groebner_matrix[(65, 179)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(17, 190)];
    groebner_matrix[(65, 190)] -= reduction_term;
    groebner_row63_000000000_f(groebner_matrix, 65);
    groebner_row64_000000000_f(groebner_matrix, 65);
    let factor = -groebner_matrix[(65, 175)];
    groebner_matrix[(65, 175)] = 0.0;
    let reduction_term = factor * groebner_matrix[(18, 182)];
    groebner_matrix[(65, 182)] -= reduction_term;
    let reduction_term = factor * groebner_matrix[(18, 187)];
    groebner_matrix[(65, 187)] -= reduction_term;
    let factor = 1.0 / groebner_matrix[(10, 169)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(10, column)];
        groebner_matrix[(10, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(11, 166)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(11, column)];
        groebner_matrix[(11, column)] = reduced_entry;
    }
}

#[inline]
fn compute_basis_stage_193(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = 1.0 / groebner_matrix[(12, 163)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(12, column)];
        groebner_matrix[(12, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(13, 151)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(13, column)];
        groebner_matrix[(13, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(14, 148)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(14, column)];
        groebner_matrix[(14, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(15, 142)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(15, column)];
        groebner_matrix[(15, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(16, 154)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(16, column)];
        groebner_matrix[(16, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(17, 172)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(17, column)];
        groebner_matrix[(17, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(18, 175)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(18, column)];
        groebner_matrix[(18, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(19, 149)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(19, column)];
        groebner_matrix[(19, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(20, 164)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(20, column)];
        groebner_matrix[(20, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(21, 167)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(21, column)];
        groebner_matrix[(21, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(22, 147)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(22, column)];
        groebner_matrix[(22, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(23, 146)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(23, column)];
        groebner_matrix[(23, column)] = reduced_entry;
    }
}

#[inline]
fn compute_basis_stage_194(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = 1.0 / groebner_matrix[(24, 145)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(24, column)];
        groebner_matrix[(24, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(25, 144)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(25, column)];
        groebner_matrix[(25, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(26, 143)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(26, column)];
        groebner_matrix[(26, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(27, 168)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(27, column)];
        groebner_matrix[(27, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(28, 150)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(28, column)];
        groebner_matrix[(28, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(29, 165)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(29, column)];
        groebner_matrix[(29, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(30, 152)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(30, column)];
        groebner_matrix[(30, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(31, 153)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(31, column)];
        groebner_matrix[(31, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(32, 155)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(32, column)];
        groebner_matrix[(32, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(33, 156)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(33, column)];
        groebner_matrix[(33, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(34, 157)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(34, column)];
        groebner_matrix[(34, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(35, 158)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(35, column)];
        groebner_matrix[(35, column)] = reduced_entry;
    }
}

#[inline]
fn compute_basis_stage_195(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = 1.0 / groebner_matrix[(36, 159)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(36, column)];
        groebner_matrix[(36, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(37, 160)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(37, column)];
        groebner_matrix[(37, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(38, 161)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(38, column)];
        groebner_matrix[(38, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(39, 162)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(39, column)];
        groebner_matrix[(39, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(45, 96)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(45, column)];
        groebner_matrix[(45, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(51, 132)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(51, column)];
        groebner_matrix[(51, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(52, 133)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(52, column)];
        groebner_matrix[(52, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(53, 134)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(53, column)];
        groebner_matrix[(53, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(54, 135)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(54, column)];
        groebner_matrix[(54, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(55, 136)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(55, column)];
        groebner_matrix[(55, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(56, 137)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(56, column)];
        groebner_matrix[(56, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(57, 138)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(57, column)];
        groebner_matrix[(57, column)] = reduced_entry;
    }
}

#[inline]
fn compute_basis_stage_196(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = 1.0 / groebner_matrix[(58, 139)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(58, column)];
        groebner_matrix[(58, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(59, 140)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(59, column)];
        groebner_matrix[(59, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(60, 141)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(60, column)];
        groebner_matrix[(60, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(61, 170)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(61, column)];
        groebner_matrix[(61, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(62, 171)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(62, column)];
        groebner_matrix[(62, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(63, 173)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(63, column)];
        groebner_matrix[(63, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(64, 174)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(64, column)];
        groebner_matrix[(64, column)] = reduced_entry;
    }
    let factor = 1.0 / groebner_matrix[(65, 176)];
    for column in 0..197 {
        let reduced_entry = factor * groebner_matrix[(65, column)];
        groebner_matrix[(65, column)] = reduced_entry;
    }
    let factor = groebner_matrix[(11, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(11, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(11, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(11, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(12, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(12, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(12, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(12, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_197(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(12, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(12, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(12, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(12, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_198(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(13, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(13, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(13, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 153)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(31, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 155)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(32, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_199(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(14, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(14, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(14, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 144)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(25, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 147)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(22, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_200(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(15, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(15, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(15, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_201(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(16, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(16, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(16, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 152)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(30, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 153)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(31, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 155)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(32, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_202(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(19, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(19, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(19, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(20, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(20, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(20, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(20, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(20, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(20, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(20, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(20, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(20, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(20, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_203(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(21, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(21, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(21, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(21, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(21, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(21, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(22, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(22, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(22, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(22, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(22, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(22, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(22, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(22, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(22, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(22, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(22, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(22, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(23, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(23, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(23, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(23, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(23, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(23, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_204(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(23, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(23, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(23, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(23, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(23, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(23, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(23, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(23, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(24, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(24, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(24, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(24, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(24, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(24, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(24, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(24, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(24, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(24, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(24, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(24, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(24, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(24, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(24, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(24, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_205(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(25, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(25, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(25, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_206(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(26, 155)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(32, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(26, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_207(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(26, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(26, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(28, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(28, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_208(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 153)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(31, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 155)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(32, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_209(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(30, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(30, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(30, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 155)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(32, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_210(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(31, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(31, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(31, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 156)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(33, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_211(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(32, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(32, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(32, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 157)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(34, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_212(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(33, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(33, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(33, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 158)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(35, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(34, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(34, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_213(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(35, 159)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(36, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(35, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(35, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 160)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(37, column)];
        groebner_matrix[(36, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(36, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(36, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_214(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(36, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(36, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(36, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(36, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(36, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(36, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(36, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 161)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(38, column)];
        groebner_matrix[(37, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(37, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(37, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(37, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(37, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(37, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(37, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(37, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_215(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(38, 162)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(39, column)];
        groebner_matrix[(38, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(38, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(38, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(38, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(38, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(38, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(38, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(38, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(38, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(38, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(38, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(39, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(39, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(39, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(39, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(39, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(39, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(39, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(39, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(39, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(39, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_216(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(45, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 132)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(51, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 133)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(52, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 134)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(53, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 135)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(54, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 136)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(55, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 137)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(56, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 138)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(57, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 139)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(58, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_217(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(45, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(45, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(45, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 133)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(52, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 134)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(53, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 135)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(54, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 136)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(55, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 137)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(56, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_218(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(51, 138)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(57, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 139)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(58, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(51, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(51, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 134)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(53, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 135)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(54, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 136)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(55, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_219(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(52, 137)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(56, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 138)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(57, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 139)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(58, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(52, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(52, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 135)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(54, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 136)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(55, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_220(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(53, 137)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(56, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 138)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(57, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 139)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(58, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(53, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(53, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 136)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(55, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 137)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(56, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_221(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(54, 138)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(57, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 139)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(58, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(54, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(54, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 137)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(56, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 138)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(57, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 139)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(58, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_222(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(55, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(55, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(55, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(56, 138)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(57, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(56, 139)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(58, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(56, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(56, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(56, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_223(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(56, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(56, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(56, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(56, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(56, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(57, 139)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(58, column)];
        groebner_matrix[(57, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(57, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(57, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(57, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(57, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(57, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(57, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(57, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(57, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(57, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(57, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(57, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(57, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(57, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(57, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_224(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(58, 140)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(59, column)];
        groebner_matrix[(58, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(58, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(58, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(58, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(58, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(58, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(58, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(58, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(58, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(58, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(58, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(58, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(58, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(59, 141)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(60, column)];
        groebner_matrix[(59, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(59, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(59, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(59, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(59, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(59, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(59, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(59, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(59, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_225(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(59, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(59, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(60, 170)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(61, column)];
        groebner_matrix[(60, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(60, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(60, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(60, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(60, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(60, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(60, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(60, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(60, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(61, 171)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(62, column)];
        groebner_matrix[(61, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(61, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(61, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(61, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(61, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(61, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(61, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(62, 173)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(63, column)];
        groebner_matrix[(62, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(62, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(62, column)] -= reduction_term;
    }
}

#[inline]
fn compute_basis_stage_226(groebner_matrix: &mut SMatrix<f64, 66, 197>) {
    let factor = groebner_matrix[(62, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(62, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(63, 174)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(64, column)];
        groebner_matrix[(63, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(63, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(63, column)] -= reduction_term;
    }
    let factor = groebner_matrix[(64, 176)];
    for column in 0..197 {
        let reduction_term = factor * groebner_matrix[(65, column)];
        groebner_matrix[(64, column)] -= reduction_term;
    }
}
