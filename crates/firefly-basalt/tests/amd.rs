use firefly_basalt::utils::amd::minimum_degree_ordering;
use serde_json::Value;
#[test]
fn original_eigen_amd_patterns() {
    let data: Value = serde_json::from_str(include_str!("reference/amd.json")).unwrap();
    for (case_index, case) in data.as_array().unwrap().iter().enumerate() {
        let indices = |name: &str| -> Vec<usize> {
            case[name]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| usize::try_from(v.as_u64().unwrap()).unwrap())
                .collect()
        };
        assert_eq!(
            minimum_degree_ordering(&indices("outer"), &indices("inner")),
            indices("permutation"),
            "case {case_index}"
        );
    }
}
