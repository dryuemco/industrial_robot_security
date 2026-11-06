# Submission package v10: resubmission of Access-2026-55984 (round 3)

Built from the committed sources of branch review/access-r3 at e5a60a2 with
scripts/review/package_round3.sh. The highlighted PDF is latexdiff against 797aeb3, the source of
Access-2026-55984 (paper/build_diff.sh, with paper/mark_new_floats.py for new and revised floats).
source/ holds main_access.tex and its 12 table files with LaTeX comments removed
(paper/strip_comments.py), the class files, logos, photos and figures; source.zip compiles on its
own to the same text as the clean PDF.

03_response_to_reviewers/ is empty until the response letter is written. 04_supplementary/ holds
round3_analysis.zip: the tracked files under analysis/round3/ of the branch, the same files as on
the branch, built deterministically by scripts/review/build_round3_supplementary.py (rebuilding
from the same commit gives the same SHA-256). The two supplementary files of the earlier rounds,
audit_v2.zip and confirmatory_run_comparison.zip, are kept on the Windows machine and are added
there; whether they are unchanged from the previous upload is to be checked there.

## Files

| File | Size (bytes) | SHA-256 |
|---|---|---|
| 01_main_manuscript/Access-2026-55984_R1_main.pdf | 1164302 | a476ca0312fbf1cfab59eb29eb8f3e25131eea1b11d6ebae2bb44b1aa5ecca72 |
| 01_main_manuscript/source/IEEEtran.cls | 281957 | da751920a317ed318b7b5cd7fa585a6cc7d28502d457856382e9be24b10a3bd7 |
| 01_main_manuscript/source/bullet.png | 430 | 754772e36aab389556f98c14f5deeb4031e792b68a766dc5f9766e76e114fffc |
| 01_main_manuscript/source/figures/fig_construct_removal.pdf | 14815 | b72ff8170405453e015b624448e2e305a12d854b6d7ff04d98f00b85d24267e9 |
| 01_main_manuscript/source/figures/fig_repair_trajectory.pdf | 16264 | 67e1cf6eb09c20c7530ead78f14c600e45bbd725f1928512846056b33f9ef258 |
| 01_main_manuscript/source/figures/fig_representation_breach.pdf | 12545 | dbcd39c89f903da9d65baed14e7797f54e5e9ff7a42bc64228b0251a6573b46f |
| 01_main_manuscript/source/figures/fig_rule_heatmap.pdf | 19477 | 646ad14e2aadc886cf873a74821e0f358f86a04e9618dc469d6cef8291ca1d5e |
| 01_main_manuscript/source/figures/fig_violation_rates.pdf | 15061 | 0e075dd1622f41e208fc427fbe0a24c49dc0f8a334fb80e802855c11cad9b5f6 |
| 01_main_manuscript/source/figures/pipeline.pdf | 32608 | 6b1b3fba20f5a6c38de821fb0cdca5af7850bc4004a61751aff7139a4e25edf9 |
| 01_main_manuscript/source/ieeeaccess.cls | 42890 | 59549e6d22ba098a6ac80f0ec3afeaeac980cf3c4d8152e44d638e9379cf3e89 |
| 01_main_manuscript/source/logo.png | 9705 | 7e04ba7f0c579f3765941680e2a19437e15a81b84a138d73c54bd4afead752cd |
| 01_main_manuscript/source/main_access.tex | 171157 | 1d45f5c2621839d6db3019b1b5bf833f1400f4d535882680a62fd125f42701ca |
| 01_main_manuscript/source/notaglinelogo.png | 3150 | 955e9a07fb35ffbc8a41597b8c6e7ec4cedf6a8652f5c73a33957044aae64900 |
| 01_main_manuscript/source/photo1.jpg | 9680 | b2b7e04663ff21552d458bd0ccf4bf72ede0cc33bee6973f691415d51a591b57 |
| 01_main_manuscript/source/photo2.jpg | 292324 | 21c93698b88726e22533231745697b6f7c056b61f9ada94f6eb955cd8559d6d6 |
| 01_main_manuscript/source/photo3.jpg | 234836 | c505af593769e62fccdb6cef133df055a0b1b165391f2fe908f37d9c8d2ab1c0 |
| 01_main_manuscript/source/tables/adversarial_table.tex | 3721 | e90047372253f8811d63d51cd8e097ffb3db536c8426ff3abd0411bb9fecb00e |
| 01_main_manuscript/source/tables/code_listings.tex | 6253 | 3bc73c1a29b6139fa3b9d0978764c01df7ee3934fb482a6d1a14d8f9466e5847 |
| 01_main_manuscript/source/tables/comparator_summary.tex | 3602 | 2fd50364d314e4646b34113bb360964932caa086a1af9e2ffd0abffd3b338ec8 |
| 01_main_manuscript/source/tables/complexity_table.tex | 1534 | df4bb1460e8eb1f33c8151b2fcff4babd546965af10fa2cb8b2c449a37a836f7 |
| 01_main_manuscript/source/tables/crossmodel_registered.tex | 2958 | c11845441598c625116816a37ca53d5e39b1e4b960be4b2bfc40d5203f840be7 |
| 01_main_manuscript/source/tables/e3_regimes.tex | 1230 | 709de870911b0292a822fa523dd6ef712048b89beffe08015a8904b02ac437f6 |
| 01_main_manuscript/source/tables/precision_audit_v2.tex | 1850 | cb68600435487442555aef86de5b310e5e28d56d75ac1c22c16be527a01bc5e2 |
| 01_main_manuscript/source/tables/prompts_appendix.tex | 8012 | 4674a40f3084e24e6e413e86ce00960f9ca335bcc2409fabd1b329fd9e7e8d32 |
| 01_main_manuscript/source/tables/quantization_summary.tex | 4070 | 2a6805c60876a74bc07b6935517bcfe79f6f98ba6b4787b8ee93ab0acf5f82d0 |
| 01_main_manuscript/source/tables/rep_reproducibility.tex | 1287 | 2d388ab2c3d2d653a0c25aa93e8c614418186701f2f248fed78551556229a0c0 |
| 01_main_manuscript/source/tables/sensitivity_e1_baseline.tex | 2641 | fa72b1dc2126f687952bb9d3ea55c751d8d84b352bfe603facca80c753b7b02b |
| 01_main_manuscript/source/tables/task_table.tex | 2340 | e26819481c3625a875141634b6940d144ba120f0fb7a28058f9938abaf1adb28 |
| 01_main_manuscript/source.zip | 797803 | 1576d2679a3c099b3595190a8d23fc2c1474ded5642eb4b539e3b02cc4dc8dec |
| 02_highlighted_pdf/Access-2026-55984_R1_highlighted.pdf | 1240584 | f12ae8977032174751c9d8151adcf00f31005b97b1a54e3639c8b69cf151f011 |
| 03_response_to_reviewers/README.md | 95 | ddf524694e2dd6429ec94403ac280ae5c7144373264b04aec08419c432b6c51a |
| 04_supplementary/round3_analysis.zip | 592356 | 32485ba7f19678a2531d8bd9331fc29ae857e82c130b2defc813a2aa853208f7 |
| portal_notes.md | 3032 | 7f7ec164fb8c83c0deef25b91dc76fd0d2e2bf684c779014791fcdc8d85553a5 |

## Members of 04_supplementary/round3_analysis.zip

| Member | Size (bytes) | SHA-256 |
|---|---|---|
| analysis/round3/README.md | 8983 | c996e13531a5ef49e4f89c7b71aa45bc2662ee3b760a61e68521cec883d1d906 |
| analysis/round3/common.py | 11748 | 733ee4dcf566a71abd9e89e114612a4cdcb2ea54f7da1e8d7bb38e1ad93aa88b |
| analysis/round3/out/a1/ir_tcp_payload.csv | 4830 | b1ee07cae56613c67aba16ebbe8571ca4d592e4af3d04a294d76924355e5da10 |
| analysis/round3/out/a1/prompt_fields.csv | 19571 | b56fb121873a25d1957732f435d7577a75ca6219d14dc29f321621cb51de6569 |
| analysis/round3/out/a1/prompt_log_evidence.json | 1803 | 826299a5c1ee33fddc41386527303216d0fe2090f81eaf7097b4197db93bc2f8 |
| analysis/round3/out/a1/prompt_quotes.md | 1101 | 0475f0e2c9074050f536778804101d4502eb78131adcf0ee29665bfbe471948b |
| analysis/round3/out/a1/sm6_clause_map.md | 7605 | 32210f35e0892f0fe9c060c2e3c1f33db02f7ea4b386e796992046a4569bdefb |
| analysis/round3/out/a1/sm6_split.csv | 1151 | 2cfb0ab0ef76d03cb8214df71ad18efe3f214baaaac2730f47d430a111753475 |
| analysis/round3/out/a1/table13_llm_check.csv | 1564 | ea5424907862fb6577465e95c186ad89432bd3ebd2f300fa3b0fe373752e5225 |
| analysis/round3/out/a1/table13_with_translator.csv | 3423 | 4b23c2144002644e8677fb5bd845784bb2eeaf0f9a05952ff5efb15c42ed4e2d |
| analysis/round3/out/a1/table13_with_translator_wide.csv | 1919 | 27595b759adf702378811a2c0451e78a4b63fd275ea5cb95ed4d14a31c5506ad |
| analysis/round3/out/a1/translator/minimal/T001_pick_place_collab.urscript | 1743 | abe1207b348750910b2c33fb9e6b3ef2025076c8b9fbde348c3cb5d0ca57d591 |
| analysis/round3/out/a1/translator/minimal/T002_linear_weld_fenced.urscript | 1866 | 2e95501e9192e17ab1c875177c67f768fc647c731b56fbb03734b1b3bd6c6520 |
| analysis/round3/out/a1/translator/minimal/T003_palletize_hybrid.urscript | 1922 | 0e21c4cc781782808a3e9b030fc9c5c45f3b4ae1de34353e0819aef025a7f5f9 |
| analysis/round3/out/a1/translator/minimal/T004_multi_wp_obstacle_collab.urscript | 1946 | f307f076c781d694f332e6f910386243b4dfceb553a0d362ab89e942f8102f00 |
| analysis/round3/out/a1/translator/minimal/T005_inspection_scan_collab.urscript | 1978 | f76f0b87d9da030f6fb748936538e8705fd1f7115a8bc9be0c1eb03edef5408a |
| analysis/round3/out/a1/translator/minimal/T006_bin_pick_suction_fenced.urscript | 1779 | dfbd7bf30857f15439b590d457ac9732c908a2b2299b4c195eb52ad0fb2f30fd |
| analysis/round3/out/a1/translator/minimal/T007_curved_weld_hybrid.urscript | 1893 | a97147b7b4c93dfe21feeea49bfb442661f6abb0e6c39f2e71e7b61bf21784e5 |
| analysis/round3/out/a1/translator/minimal/T008_heavy_depal_fenced.urscript | 1679 | f118a2a0ca96c2ef4ac1a4418aedce911a128a7a552e67aa6812406a02160802 |
| analysis/round3/out/a1/translator/minimal/T009_laser_scan_fenced.urscript | 2046 | c827479fa8a714029db3116e05ca2cc09ad2499780e2ee5191cf41b13e8c9295 |
| analysis/round3/out/a1/translator/minimal/T010_screw_assembly_hybrid.urscript | 1900 | 18e41d37ba05bdda46f712d34b434fbc2ed7a651499257c22a0220b153953333 |
| analysis/round3/out/a1/translator/minimal/T011_dual_zone_pick_hybrid.urscript | 1680 | d89d3e80fda7e8fa60b93c81cfbb3d2747bfca60fed44421095c06153f4a5e37 |
| analysis/round3/out/a1/translator/minimal/T012_spot_weld_collab.urscript | 2215 | b2521601e9dcbb6c5b27012d99c6f606d8fc8997232d6142a76fe788157bc698 |
| analysis/round3/out/a1/translator/minimal/T013_collab_suction_pallet.urscript | 1747 | 82243b44b06172f301f4b4a34c910f5ccde14af4d013344354116c603e828afa |
| analysis/round3/out/a1/translator/minimal/T014_hybrid_camera_inspect.urscript | 1715 | 45fe39c0fd3b8d79bcae212962a7e1c898c2f433e49d9f95a0c5ca541772bafc |
| analysis/round3/out/a1/translator/minimal/T015_laser_marking_fenced.urscript | 1798 | 26231d4d47ec5ad4d872c839514fd5eb905a5167f66d67bf04b4d672b80d5572 |
| analysis/round3/out/a1/translator/preamble/T001_pick_place_collab.urscript | 1891 | 13259d02ffb2c32f196520ff51bb8adf086a955bfb0892555a7022a6c1925719 |
| analysis/round3/out/a1/translator/preamble/T002_linear_weld_fenced.urscript | 2014 | e7953a0ccb4ddff59334adcf39be59e9105ccb1638020e199b4a0d4d94b69484 |
| analysis/round3/out/a1/translator/preamble/T003_palletize_hybrid.urscript | 2070 | 71bcb07e771446b61a060d0fe7a74980aecadbe17bc8d4740e82a2cd93e88398 |
| analysis/round3/out/a1/translator/preamble/T004_multi_wp_obstacle_collab.urscript | 2094 | c2a8906ff57500f7ebc3b24e34ad6c3186323809a5a9f9758a5ad91ac426921a |
| analysis/round3/out/a1/translator/preamble/T005_inspection_scan_collab.urscript | 2126 | 6d614d47990abd5189cfb57ce5fdcedbd65ccbf3f1ce70118f26266a6a5ed476 |
| analysis/round3/out/a1/translator/preamble/T006_bin_pick_suction_fenced.urscript | 1927 | 9aa0383fbfd138e783dc162652ac0cd03ccd5272783ffd5dfade6b2c7cf169e1 |
| analysis/round3/out/a1/translator/preamble/T007_curved_weld_hybrid.urscript | 2041 | adcc3c1a20ce8b72da94d885b42784239392603a0c5fee0c721383defe37ff98 |
| analysis/round3/out/a1/translator/preamble/T008_heavy_depal_fenced.urscript | 1827 | 59ac71ec7b733205789cbefff5f0cb9fb199185c44a3a6bf0672553e3076165f |
| analysis/round3/out/a1/translator/preamble/T009_laser_scan_fenced.urscript | 2194 | 4e31d7ce26d202ed5831c5d8c443cb2823f2f76caecaf2ad7b21c5fa3f320a58 |
| analysis/round3/out/a1/translator/preamble/T010_screw_assembly_hybrid.urscript | 2048 | dd2ad417cd3bc1e9f34d21dd4644b7da5255ee411bda2b084617fa70a2f00e9a |
| analysis/round3/out/a1/translator/preamble/T011_dual_zone_pick_hybrid.urscript | 1828 | b0d4c8a6c301a20c9064881583892e56b111ba0c32f916bd305ee6c6d0a5839e |
| analysis/round3/out/a1/translator/preamble/T012_spot_weld_collab.urscript | 2363 | 48c4489a37a827dff773b53b7d5f645050e3903e8f8b4afc09b0abea69aa4529 |
| analysis/round3/out/a1/translator/preamble/T013_collab_suction_pallet.urscript | 1895 | cdaeabbebff84f1aa9296b145c71e6a5df10e8880716621b2b75482c267d18bc |
| analysis/round3/out/a1/translator/preamble/T014_hybrid_camera_inspect.urscript | 1863 | 24d54a156e365440ac7eb9b94e03053f2005c907a26fcd95bd690a32dbc67dcc |
| analysis/round3/out/a1/translator/preamble/T015_laser_marking_fenced.urscript | 1946 | c40c21d6ae5b5a44f7b6af95e5780d226a4c477969f48146f919bf2239e2c22b |
| analysis/round3/out/a1/translator_check.csv | 45 | 9b3cff6abfd7fbec6a1a887517618a7bd9adba17237e8e31d38ff74f80a8d7b2 |
| analysis/round3/out/a1/translator_hashes.csv | 5708 | 2c92c72e4f2369abd6b977847f92a168830d948bd2663de2a38e7d15f115757a |
| analysis/round3/out/a2/cochran_all.csv | 4782 | c83d79d659a7d3e9b2ff0e0675c50b6208c5a53560d8b30c925c7b390b4e23bf |
| analysis/round3/out/a2/osf_search.csv | 81740 | 061a8dbbb1a61c9dd2ef8f67b661f39b6eb53c75c61dd1e7db578bcb133d90f1 |
| analysis/round3/out/a2/osf_search.md | 31148 | f68e737343294fd0719d8ddb288120a584a4aef56114d8f4c123febd77df1449 |
| analysis/round3/out/a2/table16_check.csv | 1217 | a7411e0919177f10f7447275e386c34368be52101fec989d18ca7adbea69bc8f |
| analysis/round3/out/a2/verify_counts.csv | 951 | 28d1cdbe25319db8025cc727c930cb7824218c5c73890a430b2a69da748fc34a |
| analysis/round3/out/a2/vid_four_condition_family.csv | 610 | e0b79e26e405c0367d02cd24584a38fbecd0c651465533c6346726f89b43a5ab |
| analysis/round3/out/a3/differences.csv | 1001 | 0ca756d168fc659c82fced18a1500fc113154f1cc83cd57e00c35283210486f5 |
| analysis/round3/out/a3/never_fires.md | 936 | dfee952607ea1350ea27b5082e235b0c1907b07eeaad0d51ae78d54ed74eb4c9 |
| analysis/round3/out/a3/per_rule_means.csv | 1931 | 378943d170da174b1a182e7fa3314296f9e6a2f72b157babcf1abb62ce12c170 |
| analysis/round3/out/a3/programs.csv | 23846 | 9551296e70594abb14ac0897a4635cbff2aecc8b5caba7411ad1ac53dd314133 |
| analysis/round3/out/a3/ratios.csv | 474 | c07e6b46c72e1946291d17dcbe9fddbe07e8ab269308eb6cdb91d72b738c09ae |
| analysis/round3/out/a3/reviewer_check.csv | 611 | a39dc67764b4437ffb1d1e811e840e1af4b91b50cf5ee4bff647ca281f377643 |
| analysis/round3/out/a4/audit_population.csv | 155 | e6051856f0445da4891c44f71feaf049e0955f1c4c59d906172bf76e2fbb47a2 |
| analysis/round3/out/a4/disagreements.csv | 6967 | eabc384ce1fdf8cb10fe3ca9111c13b8672516916252202e46829c7fa42d23a9 |
| analysis/round3/out/a4/gate_calls.csv | 24773 | 619804e6406354b6d9a3cbe4e04a2924e7f9597a1d6191fd41eb18c62312817f |
| analysis/round3/out/a4/precision.csv | 2213 | 1fa9d30540bf20f953d189e875b89cb7cfa5f3c646dece4ca5593633da7fc295 |
| analysis/round3/out/a4/precision_check.csv | 718 | d990f7412407c473989e138d2189d626900d22c2a3eca4144485179560a0c9c8 |
| analysis/round3/out/a4/rater_guide_quotes.md | 1017 | 2366516fb039bb7c2d645427f9b1d73a93688e26f9fada8c1873b1a335ceaee9 |
| analysis/round3/out/a4/ruleset_by_rater.csv | 1382 | 05189bef67ffcdb631c3911d5cf3764beee1d31468e2177c40295d583e1fad1b |
| analysis/round3/out/a4/safe_programs.csv | 2846 | fe355bd5d572d7f8c20dcd97f2a24b0406f5f9a51a18932c4c77b628d6ebf1d9 |
| analysis/round3/out/a4/sm2_by_location.csv | 66 | 86d6e56f9dede9c2fc7726c868a1bd8efe3ce5434b786f25c480b994843e516e |
| analysis/round3/out/a4/sm2_units.csv | 497 | b646966e1e31021d5c86c3e6d0e4b57d53806896589a47a6dcfb92deaa3272bd |
| analysis/round3/out/f1/f1_programs.csv | 35352 | 2d8588f9b5b5937f6ede490df00a12fb13436d290b873b825e8d39cee4f6f803 |
| analysis/round3/out/f1/f1_single_rule.csv | 5945 | 6ccae353e8a1918b7120ec2a8876e43f84a46cf325a20c2b436b9b14e03eb3f2 |
| analysis/round3/out/f1/f1_sm6_groups.csv | 1324 | be82ac44709e897680e21606ac50b09ba07f247d66329d213504b2e0f0531a86 |
| analysis/round3/out/f1/f2_safety_vs_baseline.csv | 1498 | 86178da8e7a64af0eded445ed180b67665525e433e4816a6031ac1f5e755ffa8 |
| analysis/round3/out/f1/f3_sm6_audit_items.csv | 3409 | 1c6d3643724fa20443ac3712923cacb342ab7e3dceb6b5bb34d982d1e4d217b7 |
| analysis/round3/out/f1/f3_sm6_audit_split.csv | 374 | e4065adacbcb95174572d6c3c8c68fef2434f78a0cbfcdb7d2d9fac62435984c |
| analysis/round3/out/f1/f4_cochran_adversarial_subtypes.csv | 6071 | ee0b4c0b5dc03c3755e29f92e87805f6cee18d8b5ab2703044f16c2bc08066c6 |
| analysis/round3/out/f1/f4_cochran_four_conditions.csv | 3311 | 37d37733acd6c8ec3a17276b7491e89a437e605b01a2e43277840a8cb1152225 |
| analysis/round3/out/f1/f4_gate_equivalence_e2_e3.csv | 101 | df916f17282c1f86db6eb5c0c343be659cf3d3e1dcc59f0b98df8601214bd74e |
| analysis/round3/out/f1/f4_locations.csv | 8487 | e238777c9daef1dadd310b923b33e6d38e837c413986435c7cb7347598d09bc3 |
| analysis/round3/out/f1/f4_locations.md | 8856 | 1c34ee24dafd28c7ffedbcdf9937f3b0e1a51a2bb79c1795235ebfd6a3c6e4b0 |
| analysis/round3/out/f1/f4_naive_vs_gatepass_cells.csv | 272 | cb64e50b18dec2620edd7ac318152b2d21908ea9f8e415bba6996cab22475d71 |
| analysis/round3/out/f1/f4_osf_cross_model.md | 2619 | 5f52fa6573c191350f042404c866f22b6d5ed4386cc7739861642b1d2ed3a25e |
| analysis/round3/out/f1/f5_table16_all_bases.csv | 893 | ebd58868bdecd5dd524643e8ec90b3ea3122ce1ee2574f6aafa7f258d014b019 |
| analysis/round3/out/f1/f6_t009.md | 5735 | c9d9ba622d6e2bb0806435cd9e9859237bd16a8011fedf37485bc21c1b8484ec |
| analysis/round3/out/f1/f7_sm6_only_safe.csv | 2876 | 366a436b3ac4f55beb216076c79189301ab649ef4cfa6eb056b0d78fd33b7086 |
| analysis/round3/out/f2/frontier_programs.csv | 20569 | 06b20f5628c8509850553d6d52300d88f5118187e668f3cb207610f4ad33ec16 |
| analysis/round3/out/f2/frontier_prompts.json | 2304 | bd6b86fa7a72b1c0369c13f6717c4bf90d64a677ec661df04a7e80e7fc5082b5 |
| analysis/round3/out/f2/frontier_sm6_groups.csv | 933 | 6f68e5d147b4cbd17815b505e9af370c0d2266a432d3d32344d6a5252de96588 |
| analysis/round3/out/flags.json | 192 | f165960739b4da451a185d71753bdad8acb226f430c8d5b4dd8db21f4fa9f306 |
| analysis/round3/out/gate_equivalence.csv | 352 | ee1f09137e14d3b693613413ce35ca1dbd820caec9610e66efb9878d12525bb9 |
| analysis/round3/out/impact/denominators.md | 9245 | 93956a7fe35e22592cf5fc5659acc27dd903e99f0818b2ddbd2e9de5512202ca |
| analysis/round3/out/impact/figures.md | 3994 | 652efb4d721dc8b14e0efd9ba32c345a9eb9f8606a0a3f8de055f6a8a9e45710 |
| analysis/round3/out/impact/hits.csv | 59703 | b5987144f17d247c7666898e309d22b64f4a18e71cdcfc4ce7fae109ebcc80b7 |
| analysis/round3/out/impact/hits.md | 63344 | 24070726780aa52faaa0b3b5742b206ed189e0284f6e08f12828216ce410e0a2 |
| analysis/round3/out/osf/registration.json | 155344 | 5c881c171a22a4ca9e2e39ab0c1569ca7dcae8cb48d7c74c9874303123707e9c |
| analysis/round3/out/osf/schema_response_69bff22a89bef16c2f3d56b9.json | 45071 | e781d6127682f866338eaf9381026f4112eb52a09cbc44eaf8171211873d1783 |
| analysis/round3/out/osf/schema_response_69d3c5c421a5eb766f7a850e.json | 75203 | c49d34eb8471e3fd5059d259715615d23f7ba0b5515553f8d6788bd211a06254 |
| analysis/round3/out/osf/schema_response_6a02e67cf944b975e74bbdbd.json | 106125 | fe29721bd8624c39348dbe63b415f25ff0af2098a9e0fb62cd2de0f533eba9a2 |
| analysis/round3/out/osf/schema_response_6a076690b12b31eaddaaca6d.json | 106291 | 7e901b891b313a68106a42fb67d193309815028eb8bd6ff2c0b6a37df74f004c |
| analysis/round3/out/osf/schema_response_6ab196ba288fe69e4d07a19f.json | 107014 | 37ae2380f3344473beb27369c457ee5e4eff6891dc5339b841d21fcbef2bfbfc |
| analysis/round3/out/osf/schema_responses.json | 440236 | 8a787cc5529afaca5ca5735ad97f75834008d842f87e0aaaefe467a7faa1b0fc |
| analysis/round3/out/provenance_copies.csv | 917 | dc56f6b97afac3921ec62e15d95767967900b4d836197d1c5e34f7353566a9dc |
| analysis/round3/out/provenance_inputs.csv | 3601 | 8bbba80a66377e68e7e8f002428a2f66ddd8fbe760282b28d8b468eee570c9b7 |
| analysis/round3/out/r3/count_check.csv | 637 | b4cd89f33c943072f851a9bc1e458a9a81cff4c23a2f29a1d10459b3c985cd6d |
| analysis/round3/out/r3/count_decomposition.csv | 2786 | 7fd52c5f5ff13a350c50f484e72b9e581ccfbf066a59aece1766bbc0482546eb |
| analysis/round3/out/r3/count_differences.csv | 3763 | bd1e4dce18690cda5a8f1df497fb52df03f70bbd1316ae73a911352ffc820a42 |
| analysis/round3/out/r3/exploratory_check.csv | 523 | d37c3050e699ca4b3cd9098b4de4de708d2d804bbb28176b644740a0886e14b8 |
| analysis/round3/out/r3/first_audit_rules.csv | 687 | 02da1ec282879400bac45cc3d832c45f428faf75249b46bc26efb9bdf8bd9e64 |
| analysis/round3/out/r3/gate_passing_violating.csv | 82 | cc1fd63c65c310fc9c04c96c74ec16c3cefd073aeec591ef38ce37937ed4e43b |
| analysis/round3/out/r3/import_manifest.csv | 527956 | c8afbc44e5e53b9b61dadd816c77b4bbb15dd7b075a658cd19634262b56ac02d |
| analysis/round3/out/r3/import_masking.csv | 145 | ba113d4a15a09e8886c3b70551bd5c7cf228effe0c86cc1f1532cd344dca3bc8 |
| analysis/round3/out/r3/numeric_tokens.csv | 7125 | d8e8d0d33c8ae0aa66e7bf8298730b38cef09cd4ae66ce13436c332d61ac08d5 |
| analysis/round3/out/r3/osf_amendment1_excerpt.md | 4688 | 6f01306e09fb60f9ad39f3fead209354e1016783fe376b80b6a1797a1ecf1285 |
| analysis/round3/out/r3/sm2_per_program.csv | 305 | 4804cc15e077a7db382c37269acdfdc97ef1e43efae8332d69f9e72970e8d7ee |
| analysis/round3/out/r3/table6_fractions.csv | 1046 | a621b78220f63506d40444892031a9f196644fe30c85f2e5462a11559cd697c8 |
| analysis/round3/out/r3/tcp_values_programs.csv | 8507 | cb8c4c428071374bc623da9c22c2881b807a396f2009060b0e113ad60248f9ac |
| analysis/round3/out/r3/tcp_values_summary.csv | 267 | 3b2536f2a8a97bfb13432dc521ddf4dc181ee1d89e9ac53449e7b44f56d935b2 |
| analysis/round3/out/tables/comparator_summary.tex | 3670 | 4ac17f4b4248ae9eb0d049dec77f5a3671d82d46310bcf9a6ee8539a0a933d63 |
| analysis/round3/out/tables/crossmodel_registered.tex | 3026 | ce4553af1d1c99fedc45aefe5712db473bee3f54c75ded8e3840e9b92af13f69 |
| analysis/round3/out/tables/precision_audit_v2.tex | 1918 | a6193267fa86fc1912e49e870a065703104ae2cc936592e9ad653d1b9adb65cd |
| analysis/round3/out/tables/quantization_summary.tex | 4138 | 859a57f6b292eb0542432258c9d84c7498cee7a1e334d0da265f30e59e57d54e |
| analysis/round3/out/tables/sensitivity_e1_baseline.tex | 2709 | 26d04b066403e40c0e38a17696dd78714f0c68353b10156a245a1089a3fc1915 |
| analysis/round3/run_all.sh | 1429 | f5973aee013321558904f29eede1f3272466bea07d05555f64cbd4cca6910500 |
| analysis/round3/s00_provenance.py | 5106 | 80b1b9a4648717b839107629ccee5f83ee264e2d6f60d9144711cec3c1207667 |
| analysis/round3/s01_a1.py | 20595 | 51b8108706bb48a7e9b6fda08a9bb2414121e6080773fba68575a44c6d398b38 |
| analysis/round3/s02_a2.py | 8280 | 68590845b328d3a1b5e092dc1c70da41b18732946611b7564dafbe6a7f28e841 |
| analysis/round3/s03_a3.py | 9056 | adc688a55ceec65e21a55a0ef2282d11fbb617fb989d7f58a85ecd7a56e80ecf |
| analysis/round3/s04_a4.py | 13729 | 1768b8a028ebd57293c5f3adb7d42f217d089aca7ae08342adc78600b1b13bc7 |
| analysis/round3/s05_impact.py | 7231 | a2071b800f0911535f6848b297d3e842a8b088d3a7208cf75ceceec2620a5c68 |
| analysis/round3/s06_osf.py | 9470 | a792201862f32778daf5a5c2913cb3300713a2d40f381d22402bddbb28b9b9d1 |
| analysis/round3/s07_followup1.py | 31063 | f54ec1f0451abe75c5eabee07f411ca989f4b91baccb66292d61870bcaac7464 |
| analysis/round3/s08_paper_numbers.py | 9437 | a257d9bee061a7c8ce9885d09049143caecbe6f1b5787b561e5b3a78aca2751f |
| analysis/round3/s09_tables.py | 20849 | d0c8b01094a62cacca12126a0c37adf00fe1534b1b371c9ad7956d3154cc5828 |
| analysis/round3/s10_osf_mirror.py | 2712 | 5edf681e63e9fbfe44b162043aa4caefc793d523cb05b497308497240730d3c5 |
| analysis/round3/s11_numeric_tokens.py | 10532 | 3e87b6b871c6a68ed2666adc26f563066e6c35184357cf743debccacb041dbb1 |
| analysis/round3/s12_frontier.py | 7503 | c347795f2d9737290fe4db060da334da313e0ce94d6deedfebba106d77fa3963 |
| analysis/round3/s13_import_runs.py | 8550 | 4e0ed5029e91b9b981994afadaa154287fc4e306b96d6a991eab55e4e531baa9 |
| analysis/round3/s14_exploratory_check.py | 3617 | 5465f0b1f703ca1a2e43bae23fb759961e3f5e75a5b8be84deec3b2ba5d48a68 |
| analysis/round3/s15_tcp_values.py | 4398 | 08edf4717653e1758cbbc306af7f676c5010f3a10c218bf5b69427525a5c6ab3 |
| analysis/round3/test_round3.py | 3917 | a0768f51f3de41aaee4f5032373ab0aa6764c35f55ccd8b7fb214ca913819d10 |
